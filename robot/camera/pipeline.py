"""
camera/pipeline.py  —  Builds the DepthAI pipeline for the OAK-D Lite.

Pipeline topology:
    MonoCamera (left)  ─┐
                         ├─► StereoDepth ──► XLinkOut("depth")
    MonoCamera (right) ─┘

    ColorCamera ─────────────────────────► XLinkOut("rgb")

Key design decisions:
  • stereo.setDepthAlign(CAM_A) warps the depth map to the RGB sensor's
    perspective so that a pixel coordinate in the RGB frame indexes directly
    into the depth frame — no reprojection arithmetic needed downstream.
  • maxSize=1, blocking=False on the host queues ensures the control loop
    always processes the *latest* frame and never falls behind.
  • HIGH_DENSITY preset + KERNEL_7x7 median filter gives robust close-range
    depth (0.3 m–5 m), ideal for restaurant obstacle avoidance.
  • setLeftRightCheck(True) removes stereo ghost edges caused by reflective
    surfaces (glass tables, tile floors, metal chair legs).
"""

import depthai as dai
import sys
import os

# Allow importing config when this file is run as __main__
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import FRAME_W, FRAME_H, CAMERA_FPS


def build_pipeline() -> dai.Pipeline:
    """
    Construct and return a configured DepthAI pipeline.

    Outputs (available on the host via device.getOutputQueue):
        "rgb"   — BGR uint8 numpy array, shape (FRAME_H, FRAME_W, 3)
        "depth" — uint16 numpy array, shape (FRAME_H, FRAME_W), units = mm
    """
    pipeline = dai.Pipeline()

    # ── RGB camera (centre sensor on OAK-D Lite) ─────────────────────────────
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    cam_rgb.setPreviewSize(FRAME_W, FRAME_H)      # preview output = small, BGR
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(CAMERA_FPS)

    # ── Mono cameras (stereo pair on OAK-D Lite) ──────────────────────────────
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)   # left sensor
    mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_left.setFps(CAMERA_FPS)

    mono_right = pipeline.create(dai.node.MonoCamera)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)  # right sensor
    mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono_right.setFps(CAMERA_FPS)

    # ── Stereo depth ──────────────────────────────────────────────────────────
    stereo = pipeline.create(dai.node.StereoDepth)
    # HIGH_DENSITY: extended disparity + SGBM algorithm — dense close-range
    #   depth coverage ideal for obstacle avoidance (depthai 2.x API)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetType.HIGH_DENSITY)
    stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
    # Left-right consistency check removes stereo artifacts on shiny surfaces
    stereo.setLeftRightCheck(True)
    # Subpixel disabled: faster, fine for metre-scale obstacle detection
    stereo.setSubpixel(False)
    # CRITICAL: align depth to CAM_A so depth[y, x] == distance at rgb[y, x]
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)

    # ── XLinkOut nodes — send streams over USB to the host ────────────────────
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")

    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")

    cam_rgb.preview.link(xout_rgb.input)
    stereo.depth.link(xout_depth.input)

    return pipeline


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    import numpy as np

    print("Building pipeline and opening OAK-D Lite …")
    pipeline = build_pipeline()

    with dai.Device(pipeline) as device:
        q_rgb   = device.getOutputQueue("rgb",   maxSize=1, blocking=False)
        q_depth = device.getOutputQueue("depth", maxSize=1, blocking=False)

        print("Capturing 10 frame pairs …")
        pairs = 0
        while pairs < 10:
            msg_rgb   = q_rgb.tryGet()
            msg_depth = q_depth.tryGet()
            if msg_rgb is None or msg_depth is None:
                continue
            rgb   = msg_rgb.getCvFrame()
            depth = msg_depth.getCvFrame()
            print(
                f"  Frame {pairs + 1}: "
                f"RGB {rgb.shape} dtype={rgb.dtype}  "
                f"Depth {depth.shape} dtype={depth.dtype}  "
                f"depth[240,320]={depth[240, 320]} mm"
            )
            pairs += 1

    print("Pipeline test complete.")
