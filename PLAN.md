# Plan: Restaurant Delivery Robot — OAK-D Lite + RPi 5 + Hiwonder Tank Chassis

## Context

Building a restaurant drink-delivery robot from scratch. The robot uses an OAK-D Lite depth camera for both obstacle avoidance (stereo depth) and target navigation (AprilTag detection on RGB). Each customer table has a unique AprilTag (tag36h11 family). An operator assigns a delivery via CLI; the robot searches for the target tag, navigates to it while avoiding obstacles, waits for pickup, then returns to the workstation (also AprilTag-marked, ID 0).

**Hardware summary:**
- Hiwonder Tank Car Chassis (2 DC motors, track drive)
- Yahboom 4-channel encoder motor driver (I2C interface via STM32 coprocessor — NOT direct GPIO)
- OAK-D Lite (USB3, RGB + stereo depth)
- Raspberry Pi 5

**Recommended AprilTag size:** Print at **15 cm × 15 cm** (tag36h11 family). Detectable 3–4 m away, which is appropriate for a restaurant floor layout.

---

## File Structure

```
robot/
├── config.py               # All tunable constants (I2C address, speeds, thresholds)
├── main.py                 # Entry point: wires components, starts CLI + main loop
├── cli.py                  # Operator CLI in daemon thread (deliver / status / stop)
├── state_machine.py        # RobotStateMachine + RobotState enum
├── camera/
│   ├── __init__.py
│   ├── pipeline.py         # DepthAI pipeline: RGB + aligned stereo depth → host
│   └── camera_manager.py   # CameraManager: device lifecycle, non-blocking frame access
├── detection/
│   ├── __init__.py
│   ├── apriltag_detector.py  # AprilTagDetector: pupil-apriltags + depth lookup
│   └── obstacle_detector.py  # ObstacleDetector: L/C/R zone depth analysis
├── motors/
│   ├── __init__.py
│   └── motor_controller.py   # MotorController: Yahboom I2C driver via smbus2
├── navigation/
│   ├── __init__.py
│   └── navigator.py          # Navigator: visual servoing → MotorCommand
└── requirements.txt
```

---

## Hardware Wiring

### OAK-D Lite → RPi 5
- USB-C (USB3) → USB-A port on RPi 5

### Yahboom Motor Driver → RPi 5
```
RPi 5 Pin 3 (GPIO 2, SDA) → Yahboom SDA
RPi 5 Pin 5 (GPIO 3, SCL) → Yahboom SCL
RPi 5 Pin 6 (GND)         → Yahboom GND
RPi 5 Pin 4 (5V)          → Yahboom 5V (logic only)
External battery (7.4V)   → Yahboom motor power terminals
```

Enable I2C on RPi 5: `sudo raspi-config` → Interface Options → I2C → Enable

### Verify I2C connection
```bash
sudo i2cdetect -y 1
# Should show Yahboom module address (likely 0x34)
# Update MOTOR_I2C_ADDR in config.py if different
```

---

## State Machine

```
IDLE ──assign_delivery(n)──► SEARCHING ──tag found──► APPROACHING
                                  ▲                         │
                            tag lost ◄─────────────────────┘ (if tag disappears)
                                                             │ arrived
                                                        DELIVERING (wait 5s)
                                                             │
                                                      RETURNING (target=tag 0)
                                                             │ arrived at workstation
                                                           IDLE
```

---

## Verification Plan

### Phase 1 — Camera only (on RPi 5)
```bash
pip install -r requirements.txt
python -c "import depthai as dai; print(dai.__version__)"
python robot/camera/camera_manager.py  # standalone test prints frame shapes
```

### Phase 2 — Motor test (no camera)
```bash
python robot/motors/motor_controller.py  # standalone: forward 2s, rotate 1s, stop
# Verify correct direction; negate MOTOR_LEFT_REVERSED or MOTOR_RIGHT_REVERSED in config.py if needed
```

### Phase 3 — AprilTag detection test
```bash
python robot/detection/apriltag_detector.py
# Hold a printed 15cm tag36h11 in front of camera at 1m
# Expected: TagDetection(tag_id=X, center_x~320, center_y~240, distance_m~1.0)
```

### Phase 4 — Obstacle detection test
```bash
python robot/detection/obstacle_detector.py
# Place hand in front of camera at 60cm
# Expected: center=BLOCKED, sides=CLEAR
```

### Phase 5 — Full integration
```bash
python robot/main.py
# > deliver 1
# Robot searches for tag 1, drives to it, stops ~55cm away, returns to tag 0
```
