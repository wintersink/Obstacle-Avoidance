"""
detection/apriltag_detector.py  —  AprilTag detection with depth enrichment.

Uses the pupil-apriltags library (Python bindings for the C apriltag library).
Each detection is enriched with the tag's distance in metres by sampling the
5×5 pixel neighbourhood around the tag's centre in the aligned depth frame.

Install: pip install pupil-apriltags
"""

from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np
import apriltag  # installed as part of pupil-apriltags

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import APRILTAG_FAMILY, APRILTAG_QUAD_DECIMATE, FRAME_W, FRAME_H


@dataclass
class TagDetection:
    """A single detected AprilTag with its position and distance."""
    tag_id:   int    # AprilTag numeric ID
    center_x: float  # Pixel X coordinate in the RGB frame
    center_y: float  # Pixel Y coordinate in the RGB frame
    distance_m: float  # Distance from camera in metres (0.0 = too close to measure)


class AprilTagDetector:
    """
    Detects tag36h11 AprilTags in BGR frames and returns TagDetection objects
    enriched with distance data from the aligned depth frame.

    Parameters:
        quad_decimate — downsample factor before quad detection.
                        2.0 gives ~2× speedup with negligible accuracy loss
                        for 15 cm tags.  Use 1.0 for maximum sensitivity.
        nthreads      — CPU threads for the detector.  2 is a good default
                        for Raspberry Pi 4/5 (leaves cores free for the main loop).
    """

    def __init__(
        self,
        family: str = APRILTAG_FAMILY,
        quad_decimate: float = APRILTAG_QUAD_DECIMATE,
        nthreads: int = 2,
    ) -> None:
        self._detector = apriltag.Detector(
            families=family,
            nthreads=nthreads,
            quad_decimate=quad_decimate,
            quad_sigma=0.0,      # no Gaussian blur before quad detection
            refine_edges=1,      # improve corner localisation
            decode_sharpening=0.25,
        )

    def detect(
        self, bgr: np.ndarray, depth_mm: np.ndarray
    ) -> list[TagDetection]:
        """
        Run detection on a BGR frame and enrich each result with depth.

        Args:
            bgr      — uint8 array (H, W, 3) from CameraManager.get_frames()
            depth_mm — uint16 array (H, W) from CameraManager.get_frames()
                       Same pixel coordinates as bgr because of depth alignment.

        Returns:
            List of TagDetection, one per detected tag.  May be empty.
        """
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        raw_detections = self._detector.detect(gray)

        results: list[TagDetection] = []
        h, w = depth_mm.shape

        for det in raw_detections:
            cx = float(det.center[0])
            cy = float(det.center[1])

            # Sample a 5×5 patch around the tag centre.
            # This handles the common case where the depth value at the exact
            # centre pixel is 0 (invalid) due to the stereo minimum range limit
            # when the robot is very close to the tag.
            ix = int(round(cx))
            iy = int(round(cy))
            x0, x1 = max(0, ix - 2), min(w, ix + 3)
            y0, y1 = max(0, iy - 2), min(h, iy + 3)
            patch = depth_mm[y0:y1, x0:x1]
            valid = patch[patch > 0]

            if valid.size == 0:
                # No valid depth in patch → robot is very close to the tag;
                # treat distance as 0.0 so is_arrived() returns True.
                distance_m = 0.0
            else:
                distance_m = float(np.min(valid)) / 1000.0

            results.append(TagDetection(
                tag_id=det.tag_id,
                center_x=cx,
                center_y=cy,
                distance_m=distance_m,
            ))

        return results


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    import time
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
    from robot.camera.camera_manager import CameraManager

    camera   = CameraManager()
    detector = AprilTagDetector()
    camera.start()

    print("Scanning for AprilTags …  (Ctrl-C to stop)")
    print("Hold a printed 15 cm tag36h11 ~1 m in front of the camera.")
    try:
        while True:
            bgr, depth = camera.get_frames()
            if bgr is None or depth is None:
                continue
            detections = detector.detect(bgr, depth)
            if detections:
                for d in detections:
                    print(
                        f"  Tag {d.tag_id:3d} | "
                        f"centre ({d.center_x:6.1f}, {d.center_y:6.1f}) | "
                        f"distance {d.distance_m:.3f} m"
                    )
            else:
                print("  (no tags detected)")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        camera.stop()
