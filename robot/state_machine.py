"""
state_machine.py  —  Central coordinator for the restaurant delivery robot.

RobotStateMachine owns the control loop, holds all perception and actuation
components, and implements the delivery state machine:

    IDLE        → waiting for a delivery assignment
    SEARCHING   → rotating in place to locate the target AprilTag
    APPROACHING → driving toward the tag, avoiding obstacles
    DELIVERING  → stopped at the table, waiting for drink pickup
    RETURNING   → navigating back to the workstation (tag ID 0)

The CLI thread calls assign_delivery() and emergency_stop() from a separate
thread.  All shared state is protected with a threading.Lock.

Control loop rate:  ~CAMERA_FPS  (driven by frame availability, not sleep)
"""

from __future__ import annotations

import logging
import threading
import time
from enum import Enum, auto

from camera.camera_manager import CameraManager
from detection.apriltag_detector import AprilTagDetector, TagDetection
from detection.obstacle_detector import ObstacleDetector
from motors.motor_controller import MotorController
from navigation.navigator import Navigator
from config import (
    CAMERA_FPS,
    DELIVER_WAIT_S,
    SEARCH_TIMEOUT_S,
    WORKSTATION_TAG_ID,
)

logger = logging.getLogger(__name__)


class RobotState(Enum):
    IDLE        = auto()
    SEARCHING   = auto()
    APPROACHING = auto()
    DELIVERING  = auto()
    RETURNING   = auto()


class RobotStateMachine:
    """
    Main robot coordinator.

    Usage:
        sm = RobotStateMachine(camera, tag_det, obs_det, navigator, motors)
        cli.start()      # CLI thread calls sm.assign_delivery() etc.
        camera.start()
        sm.run()         # blocks until KeyboardInterrupt or sm.shutdown()
    """

    def __init__(
        self,
        camera:    CameraManager,
        tag_det:   AprilTagDetector,
        obs_det:   ObstacleDetector,
        navigator: Navigator,
        motors:    MotorController,
    ) -> None:
        self._camera    = camera
        self._tag_det   = tag_det
        self._obs_det   = obs_det
        self._navigator = navigator
        self._motors    = motors

        self._lock       = threading.Lock()
        self._state      = RobotState.IDLE
        self._target_id: int | None = None     # AprilTag ID to deliver to
        self._search_start: float   = 0.0      # time.monotonic() when SEARCHING began
        self._deliver_start: float  = 0.0      # time.monotonic() when DELIVERING began
        self._stop_event = threading.Event()

    # ── Public API (called from CLI thread) ───────────────────────────────────

    @property
    def current_state(self) -> RobotState:
        with self._lock:
            return self._state

    def assign_delivery(self, table_id: int) -> bool:
        """
        Assign a delivery order.  Only accepted when the robot is IDLE.

        Args:
            table_id — AprilTag ID of the destination table

        Returns:
            True if accepted, False if the robot is currently busy.
        """
        with self._lock:
            if self._state != RobotState.IDLE:
                logger.warning(
                    "Delivery to table %d rejected — robot is %s",
                    table_id, self._state.name,
                )
                return False
            self._target_id    = table_id
            self._search_start = time.monotonic()
            self._state        = RobotState.SEARCHING
            logger.info("Delivery assigned: table %d → SEARCHING", table_id)
            return True

    def emergency_stop(self) -> None:
        """Immediately stop motors and return to IDLE from any state."""
        with self._lock:
            self._motors.stop()
            self._state     = RobotState.IDLE
            self._target_id = None
            logger.warning("Emergency stop — returned to IDLE")

    def shutdown(self) -> None:
        """Signal the run() loop to exit cleanly."""
        self._stop_event.set()

    # ── Main loop ─────────────────────────────────────────────────────────────

    def run(self) -> None:
        """
        Blocking control loop.  Call this from the main thread after
        camera.start().  Exits on shutdown() or KeyboardInterrupt.
        """
        frame_period = 1.0 / CAMERA_FPS
        logger.info("Robot ready.  State: IDLE")

        while not self._stop_event.is_set():
            loop_start = time.monotonic()
            self._tick()
            elapsed = time.monotonic() - loop_start
            sleep_for = frame_period - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)

    def _tick(self) -> None:
        """Execute one iteration of the control loop."""
        with self._lock:
            state = self._state

        if state == RobotState.IDLE:
            self._handle_idle()
        elif state == RobotState.SEARCHING:
            self._handle_searching()
        elif state == RobotState.APPROACHING:
            self._handle_approaching()
        elif state == RobotState.DELIVERING:
            self._handle_delivering()
        elif state == RobotState.RETURNING:
            self._handle_returning()

    # ── State handlers ────────────────────────────────────────────────────────

    def _handle_idle(self) -> None:
        self._motors.stop()

    def _handle_searching(self) -> None:
        bgr, depth = self._camera.get_frames()
        if bgr is None or depth is None:
            return

        with self._lock:
            target_id   = self._target_id
            search_start = self._search_start

        elapsed = time.monotonic() - search_start

        # Check for timeout
        if elapsed > SEARCH_TIMEOUT_S:
            logger.warning(
                "Search timeout after %.0f s — returning to IDLE", elapsed
            )
            with self._lock:
                self._motors.stop()
                self._state     = RobotState.IDLE
                self._target_id = None
            return

        detections = self._tag_det.detect(bgr, depth)
        target = _find_tag(detections, target_id)

        if target is not None:
            logger.info(
                "Tag %d found at (%.0f, %.0f) %.2f m — APPROACHING",
                target_id, target.center_x, target.center_y, target.distance_m,
            )
            with self._lock:
                self._state = RobotState.APPROACHING
            return

        cmd = self._navigator.compute_search(elapsed)
        self._motors.set_speeds(cmd.left, cmd.right)
        logger.debug("SEARCHING: %s  elapsed=%.1f s", cmd.reason, elapsed)

    def _handle_approaching(self) -> None:
        bgr, depth = self._camera.get_frames()
        if bgr is None or depth is None:
            return

        with self._lock:
            target_id = self._target_id

        detections = self._tag_det.detect(bgr, depth)
        target     = _find_tag(detections, target_id)
        obstacles  = self._obs_det.analyze(depth)

        # Tag lost → fall back to SEARCHING
        if target is None:
            logger.info("Tag %d lost — back to SEARCHING", target_id)
            with self._lock:
                self._state        = RobotState.SEARCHING
                self._search_start = time.monotonic()
            return

        # Arrived at destination
        if self._navigator.is_arrived(target):
            logger.info(
                "Arrived at tag %d (%.2f m) — DELIVERING", target_id, target.distance_m
            )
            with self._lock:
                self._motors.stop()
                self._state         = RobotState.DELIVERING
                self._deliver_start = time.monotonic()
            return

        cmd = self._navigator.compute_approach(target, obstacles)
        self._motors.set_speeds(cmd.left, cmd.right)
        logger.debug(
            "APPROACHING: %s  tag=%.2f m  obstacles=%s",
            cmd.reason, target.distance_m, obstacles,
        )

    def _handle_delivering(self) -> None:
        self._motors.stop()

        with self._lock:
            deliver_start = self._deliver_start

        elapsed = time.monotonic() - deliver_start
        if elapsed >= DELIVER_WAIT_S:
            logger.info(
                "Delivery complete after %.1f s — RETURNING to workstation (tag %d)",
                elapsed, WORKSTATION_TAG_ID,
            )
            with self._lock:
                self._target_id    = WORKSTATION_TAG_ID
                self._search_start = time.monotonic()
                self._state        = RobotState.RETURNING

    def _handle_returning(self) -> None:
        bgr, depth = self._camera.get_frames()
        if bgr is None or depth is None:
            return

        detections = self._tag_det.detect(bgr, depth)
        target     = _find_tag(detections, WORKSTATION_TAG_ID)
        obstacles  = self._obs_det.analyze(depth)

        # Workstation tag lost → search for it
        if target is None:
            with self._lock:
                elapsed = time.monotonic() - self._search_start

            cmd = self._navigator.compute_search(elapsed)
            self._motors.set_speeds(cmd.left, cmd.right)

            if elapsed > SEARCH_TIMEOUT_S:
                logger.warning("Could not find workstation tag — stopping")
                with self._lock:
                    self._motors.stop()
                    self._state     = RobotState.IDLE
                    self._target_id = None
            return

        # Arrived back at workstation
        if self._navigator.is_arrived(target):
            logger.info("Returned to workstation — IDLE")
            with self._lock:
                self._motors.stop()
                self._state     = RobotState.IDLE
                self._target_id = None
            return

        cmd = self._navigator.compute_approach(target, obstacles)
        self._motors.set_speeds(cmd.left, cmd.right)
        logger.debug(
            "RETURNING: %s  workstation=%.2f m  obstacles=%s",
            cmd.reason, target.distance_m, obstacles,
        )


# ── Helper ────────────────────────────────────────────────────────────────────

def _find_tag(
    detections: list[TagDetection], tag_id: int | None
) -> TagDetection | None:
    """Return the first detection matching tag_id, or None."""
    if tag_id is None:
        return None
    for det in detections:
        if det.tag_id == tag_id:
            return det
    return None
