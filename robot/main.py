"""
main.py  —  Entry point for the restaurant delivery robot.

Run on the Raspberry Pi (RPi 4 or RPi 5):
    cd robot/
    python main.py

The script:
    1. Wires together all components (camera, detectors, motors, navigator)
    2. Starts the operator CLI in a background thread
    3. Opens the OAK-D Lite and starts the DepthAI pipeline
    4. Runs the control loop until Ctrl-C or 'quit' in the CLI
    5. Shuts down motors and camera cleanly on exit

Prerequisites:
    pip install -r requirements.txt
    sudo raspi-config  →  Interface Options → I2C → Enable
    sudo i2cdetect -y 1  →  verify Yahboom motor driver appears (e.g. 0x34)
"""

import logging
import sys
import os

# ── Logging setup  ────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(name)s  %(message)s",
    datefmt="%H:%M:%S",
)

# ── Ensure the robot package directory is on sys.path ─────────────────────────
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from camera.camera_manager import CameraManager
from detection.apriltag_detector import AprilTagDetector
from detection.obstacle_detector import ObstacleDetector
from motors.motor_controller import MotorController
from navigation.navigator import Navigator
from state_machine import RobotStateMachine
from cli import DeliveryCLI


def main() -> None:
    # ── Construct components ──────────────────────────────────────────────────
    camera    = CameraManager()
    motors    = MotorController()   # opens I2C bus and configures motor type
    tag_det   = AprilTagDetector()
    obs_det   = ObstacleDetector()
    navigator = Navigator()

    sm  = RobotStateMachine(camera, tag_det, obs_det, navigator, motors)
    cli = DeliveryCLI(sm)

    # ── Start CLI thread ──────────────────────────────────────────────────────
    cli.start()

    # ── Open camera and run ───────────────────────────────────────────────────
    camera.start()
    try:
        sm.run()   # blocks until sm.shutdown() or KeyboardInterrupt
    except KeyboardInterrupt:
        print("\nCtrl-C received — shutting down …")
    finally:
        motors.cleanup()
        camera.stop()
        print("Robot stopped cleanly.")


if __name__ == "__main__":
    main()
