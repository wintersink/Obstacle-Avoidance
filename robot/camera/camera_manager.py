"""
camera/camera_manager.py  —  OAK-D Lite device lifecycle and frame access.

CameraManager owns the DepthAI Device object and exposes non-blocking frame
retrieval.  maxSize=1, blocking=False on both output queues ensures the control
loop always receives the most recent frame rather than stale buffered data.
"""

from __future__ import annotations

import numpy as np
import depthai as dai
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from camera.pipeline import build_pipeline
from config import FRAME_W, FRAME_H


class CameraManager:
    """
    Manages the OAK-D Lite device: opens the USB connection, starts the
    DepthAI pipeline, and provides non-blocking access to RGB and depth frames.

    Usage:
        camera = CameraManager()
        camera.start()
        rgb, depth = camera.get_frames()   # either may be None if no new frame
        camera.stop()
    """

    def __init__(self) -> None:
        self._pipeline = build_pipeline()
        self._device: dai.Device | None = None
        self._q_rgb:   dai.DataOutputQueue | None = None
        self._q_depth: dai.DataOutputQueue | None = None

    def start(self) -> None:
        """Open the OAK-D Lite, start the pipeline, connect output queues."""
        self._device = dai.Device(self._pipeline)
        # maxSize=1, blocking=False → always return the latest frame;
        # if the host is slow, older frames are silently discarded.
        self._q_rgb   = self._device.getOutputQueue("rgb",   maxSize=1, blocking=False)
        self._q_depth = self._device.getOutputQueue("depth", maxSize=1, blocking=False)
        print("[Camera] OAK-D Lite started.")

    def get_frames(self) -> tuple[np.ndarray | None, np.ndarray | None]:
        """
        Return the latest (bgr, depth_mm) pair.

        Returns:
            bgr      — uint8 numpy array shape (FRAME_H, FRAME_W, 3), or None
            depth_mm — uint16 numpy array shape (FRAME_H, FRAME_W), units mm, or None
                       Zero pixels represent invalid / missing depth measurements.
        """
        if self._q_rgb is None or self._q_depth is None:
            raise RuntimeError("CameraManager.start() must be called before get_frames().")

        msg_rgb   = self._q_rgb.tryGet()
        msg_depth = self._q_depth.tryGet()

        bgr      = msg_rgb.getCvFrame()   if msg_rgb   is not None else None
        depth_mm = msg_depth.getCvFrame() if msg_depth is not None else None

        return bgr, depth_mm

    def stop(self) -> None:
        """Close the device and release USB resources."""
        if self._device is not None:
            self._device.close()
            self._device = None
            print("[Camera] OAK-D Lite stopped.")


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    camera = CameraManager()
    camera.start()

    print(f"Capturing frames (expected RGB {FRAME_H}×{FRAME_W}, depth {FRAME_H}×{FRAME_W}) …")
    collected = 0
    while collected < 10:
        bgr, depth = camera.get_frames()
        if bgr is None or depth is None:
            continue
        centre_depth = depth[FRAME_H // 2, FRAME_W // 2]
        print(
            f"  Frame {collected + 1}: "
            f"rgb={bgr.shape}  depth={depth.shape}  "
            f"centre={centre_depth} mm"
        )
        collected += 1

    camera.stop()
    print("Camera test complete.")
