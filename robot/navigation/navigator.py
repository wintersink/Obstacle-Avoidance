"""
navigation/navigator.py  —  Visual-servoing navigation controller.

The Navigator is a pure function-style controller: it takes perception inputs
(a target AprilTag detection + current obstacle status) and returns a
MotorCommand.  It owns no state; the RobotStateMachine drives it.

Control priority (highest → lowest):
    1. Obstacle avoidance  — overrides everything when the path is blocked
    2. Visual servoing     — proportional steering toward target tag centre
    3. Straight ahead      — when centre is clear but no tag is visible

Steering logic:
    The pixel error between the tag's centre_x and the frame centre is
    converted to a speed differential between the two tracks:

        error_px  = tag.center_x - (FRAME_W / 2)
        correction = clamp(Kp × |error_px|, 0, base_speed - MIN_SPEED)
        if error_px > 0 (tag to the right): left track faster
        if error_px < 0 (tag to the left):  right track faster

    This produces smooth proportional turning.  Tune STEER_KP in config.py:
      too high → oscillating / jerky approach
      too low  → slow correction, robot may over-shoot turns
"""

from __future__ import annotations

from dataclasses import dataclass
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    FRAME_W,
    SPEED_FORWARD,
    SPEED_TURN,
    SPEED_REVERSE,
    SPEED_SEARCH,
    TAG_CENTER_DEADBAND_PX,
    STEER_KP,
    APPROACH_STOP_DIST_M,
)
from detection.obstacle_detector import ObstacleStatus, Zone
from detection.apriltag_detector import TagDetection


# Minimum per-track speed when steering to avoid stalling a slow track
_MIN_TRACK_SPEED = 10


@dataclass
class MotorCommand:
    """A motor command for both tracks."""
    left:   int    # -100 to 100
    right:  int    # -100 to 100
    reason: str    # human-readable log string


class Navigator:
    """
    Converts perception data into tank-drive motor commands.

    Typical usage (called from RobotStateMachine.tick):
        cmd = navigator.compute_approach(target_tag, obstacle_status)
        motors.set_speeds(cmd.left, cmd.right)
    """

    def compute_approach(
        self,
        tag: TagDetection | None,
        obstacles: ObstacleStatus,
    ) -> MotorCommand:
        """
        Compute a motor command for approaching the target tag.

        Priority:
            1. All zones blocked → reverse to create escape room
            2. Centre blocked    → turn toward the clearer side
            3. Centre clear      → visual-servo toward tag (or straight if no tag)

        Args:
            tag       — current detection of the target tag, or None if not visible
            obstacles — ObstacleStatus from ObstacleDetector.analyze()
        """
        # ── Priority 1: escape when completely surrounded ─────────────────────
        if obstacles.all_blocked:
            return MotorCommand(
                -SPEED_REVERSE, -SPEED_REVERSE, "reverse: all zones blocked"
            )

        # ── Priority 2: avoid centre obstacle ────────────────────────────────
        if obstacles.center == Zone.BLOCKED:
            if obstacles.right == Zone.CLEAR:
                # Turn right in place: left track forward, right track backward
                return MotorCommand(
                    SPEED_TURN, -SPEED_TURN, "turn right: centre blocked, right clear"
                )
            else:
                # Turn left in place: left track backward, right track forward
                return MotorCommand(
                    -SPEED_TURN, SPEED_TURN, "turn left: centre blocked, left clear"
                )

        # ── Priority 3: centre is clear — visual servo toward tag ─────────────
        if tag is None:
            # No tag visible but path is clear: drive forward cautiously
            return MotorCommand(
                SPEED_FORWARD, SPEED_FORWARD, "forward: centre clear, no tag"
            )

        error_px = tag.center_x - (FRAME_W / 2)

        if abs(error_px) <= TAG_CENTER_DEADBAND_PX:
            # Tag is centred — drive straight toward it
            return MotorCommand(
                SPEED_FORWARD, SPEED_FORWARD, "forward: tag centred"
            )

        # Proportional correction
        correction = int(STEER_KP * abs(error_px) * SPEED_FORWARD)
        correction = min(correction, SPEED_FORWARD - _MIN_TRACK_SPEED)

        if error_px > 0:
            # Tag is right of centre → slow down right track to steer right
            return MotorCommand(
                SPEED_FORWARD,
                SPEED_FORWARD - correction,
                f"steer right: error={error_px:.0f}px",
            )
        else:
            # Tag is left of centre → slow down left track to steer left
            return MotorCommand(
                SPEED_FORWARD - correction,
                SPEED_FORWARD,
                f"steer left: error={error_px:.0f}px",
            )

    def compute_search(self, elapsed_s: float) -> MotorCommand:
        """
        Rotate in place to search for the target tag.

        Alternates direction every 3 seconds to sweep a full circle.

        Args:
            elapsed_s — seconds since the robot entered SEARCHING state
        """
        # Alternate direction every 3 seconds: 0–3 s clockwise, 3–6 s counter, …
        clockwise = (int(elapsed_s / 3) % 2) == 0
        if clockwise:
            return MotorCommand(SPEED_SEARCH, -SPEED_SEARCH, "search: rotating clockwise")
        else:
            return MotorCommand(-SPEED_SEARCH, SPEED_SEARCH, "search: rotating counter-clockwise")

    def is_arrived(self, tag: TagDetection | None) -> bool:
        """
        Return True when the robot has reached the delivery destination.

        Arrival is declared when:
          • The tag is visible AND its depth is ≤ APPROACH_STOP_DIST_M, OR
          • The depth patch returned 0 (too close for stereo to measure —
            the robot is essentially at the tag).
        """
        if tag is None:
            return False
        return tag.distance_m == 0.0 or tag.distance_m <= APPROACH_STOP_DIST_M
