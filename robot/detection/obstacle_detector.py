"""
detection/obstacle_detector.py  —  Reactive obstacle detection via stereo depth.

The depth frame (aligned to the RGB sensor) is divided into three equal
horizontal zones: left, centre, right.  Each zone is classified as CLEAR or
BLOCKED based on the 10th-percentile depth of valid (non-zero) pixels.

Why the 10th percentile?
    A mean or median would ignore narrow obstacles (chair legs, table legs,
    human ankles) that occupy only 5–15 % of a zone's pixels.  The 10th
    percentile catches these small but dangerous obstacles while still being
    robust to the noisy edge pixels that stereo cameras produce around depth
    discontinuities.

Zone layout (depth frame, width W):
    ┌──────────────┬────────────────────┬──────────────┐
    │  LEFT ZONE   │    CENTRE ZONE     │  RIGHT ZONE  │
    │   [0..W/3]   │   [W/3..2W/3]     │  [2W/3..W]   │
    └──────────────┴────────────────────┴──────────────┘
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

import numpy as np

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import OBSTACLE_THRESHOLD_M


class Zone(Enum):
    CLEAR   = "clear"
    BLOCKED = "blocked"


@dataclass
class ObstacleStatus:
    """Per-zone obstacle classification for the current depth frame."""
    left:   Zone
    center: Zone
    right:  Zone

    @property
    def all_blocked(self) -> bool:
        return (
            self.left   == Zone.BLOCKED
            and self.center == Zone.BLOCKED
            and self.right  == Zone.BLOCKED
        )

    @property
    def all_clear(self) -> bool:
        return (
            self.left   == Zone.CLEAR
            and self.center == Zone.CLEAR
            and self.right  == Zone.CLEAR
        )

    def __str__(self) -> str:
        def sym(z: Zone) -> str:
            return "✓" if z == Zone.CLEAR else "✗"
        return f"L={sym(self.left)} C={sym(self.center)} R={sym(self.right)}"


class ObstacleDetector:
    """
    Classifies left / centre / right depth zones as CLEAR or BLOCKED.

    Instantiate once; call analyze() on every depth frame.
    """

    # Minimum number of valid depth pixels required in a zone to trust the
    # reading.  Below this threshold the zone is conservatively BLOCKED.
    _MIN_VALID_PIXELS = 50

    def analyze(self, depth_mm: np.ndarray) -> ObstacleStatus:
        """
        Classify obstacles in a depth frame.

        Args:
            depth_mm — uint16 numpy array, shape (H, W), depth in millimetres.
                       Zero values are treated as invalid / missing readings.

        Returns:
            ObstacleStatus with per-zone Zone enum values.
        """
        w     = depth_mm.shape[1]
        third = w // 3

        return ObstacleStatus(
            left   = self._classify_zone(depth_mm[:, :third]),
            center = self._classify_zone(depth_mm[:, third : 2 * third]),
            right  = self._classify_zone(depth_mm[:, 2 * third :]),
        )

    def _classify_zone(self, region: np.ndarray) -> Zone:
        """
        Classify a single depth zone as CLEAR or BLOCKED.

        Uses the 10th percentile of valid (non-zero) depth values to detect
        narrow obstacles that would be invisible in a mean or median.
        """
        valid = region[region > 0]

        if valid.size < self._MIN_VALID_PIXELS:
            # Too few valid depth readings — fail safe: treat as BLOCKED
            return Zone.BLOCKED

        threshold_mm = OBSTACLE_THRESHOLD_M * 1000.0
        p10 = float(np.percentile(valid, 10))

        return Zone.BLOCKED if p10 < threshold_mm else Zone.CLEAR


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    import time
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    from robot.camera.camera_manager import CameraManager

    camera   = CameraManager()
    detector = ObstacleDetector()
    camera.start()

    print(
        f"Obstacle detection test (threshold = {OBSTACLE_THRESHOLD_M} m)\n"
        "Place your hand at ~60 cm in front of the camera.\n"
        "Ctrl-C to stop.\n"
    )
    try:
        while True:
            _, depth = camera.get_frames()
            if depth is None:
                continue
            status = detector.analyze(depth)
            print(f"  {status}")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
