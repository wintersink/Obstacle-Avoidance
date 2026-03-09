# ─────────────────────────────────────────────────────────────────────────────
# config.py  —  All tunable constants for the restaurant delivery robot.
#
# Change values here to tune behaviour without touching any other file.
# ─────────────────────────────────────────────────────────────────────────────

# ── I2C / Yahboom Quad-MD motor driver ───────────────────────────────────────
# The Yahboom 4-channel encoder motor driver uses an STM32 coprocessor that
# accepts motor commands over I2C.  No GPIO PWM wiring is required.
#
# Wiring (Raspberry Pi GPIO header — same pin numbers on RPi 4 and RPi 5):
#   Pin 3  (GPIO 2, SDA)  →  Yahboom SDA
#   Pin 5  (GPIO 3, SCL)  →  Yahboom SCL
#   Pin 6  (GND)          →  Yahboom GND
#   Pin 4  (5V)           →  Yahboom 5V  (logic power only)
#   External battery      →  Yahboom motor power terminals (7.4 V recommended)
#
# Enable I2C on RPi 4/5:  sudo raspi-config → Interface Options → I2C → Enable
# Verify address:       sudo i2cdetect -y 1
I2C_BUS         = 1       # /dev/i2c-1 on Raspberry Pi (pins 3 & 5)
MOTOR_I2C_ADDR  = 0x34    # Default Yahboom Quad-MD address — confirm with i2cdetect
MOTOR_TYPE      = 1       # Yahboom motor-type preset: 1=TT, 2=310, 3=520, 4=JGB37, 5=custom
                          # Check yahboom.net/study/Quad-MD-Module for your motor type

# Tank drive channel assignment (1–4, using 2 of the 4 available channels)
MOTOR_LEFT_CH   = 1       # Yahboom channel wired to left track motor
MOTOR_RIGHT_CH  = 2       # Yahboom channel wired to right track motor

# Direction inversion flags — set to True if a track runs backward on first test
MOTOR_LEFT_REVERSED  = False
MOTOR_RIGHT_REVERSED = False

# ── Motor speeds (integer, sent to Yahboom driver; valid range 0–100) ─────────
# Yahboom accepts signed values: positive = forward, negative = reverse.
# Start conservatively and increase once the robot is running correctly.
SPEED_FORWARD   = 60      # Cruise speed when approaching a tag
SPEED_TURN      = 50      # In-place turn speed (obstacle avoidance / search)
SPEED_REVERSE   = 40      # Reverse speed when all three zones are blocked
SPEED_SEARCH    = 40      # Rotation speed while scanning for target tag

# ── Navigation thresholds ─────────────────────────────────────────────────────
APPROACH_STOP_DIST_M   = 0.55   # Stop when tag depth drops below this (metres)
TAG_CENTER_DEADBAND_PX = 40     # Pixel tolerance either side of frame centre
                                 # before proportional steering correction kicks in
STEER_KP               = 0.15   # Proportional gain:  correction = Kp × |error| × base_speed
                                 # Increase → more aggressive steering; decrease → smoother
SEARCH_TIMEOUT_S       = 30.0   # Abort search and return to IDLE after this many seconds
DELIVER_WAIT_S         = 5.0    # Seconds to wait at table for drink to be picked up

# ── Obstacle detection ────────────────────────────────────────────────────────
# The depth frame is split into left / centre / right thirds.
# A zone is BLOCKED when its 10th-percentile depth < OBSTACLE_THRESHOLD_M.
# Using the 10th percentile (not the mean) ensures narrow obstacles such as
# chair legs or table legs — which occupy only a small fraction of a zone —
# are still detected reliably.
OBSTACLE_THRESHOLD_M = 0.80     # Metres; increase to give the robot more clearance

# ── Camera ────────────────────────────────────────────────────────────────────
FRAME_W     = 640   # RGB preview width  (pixels) — must match pipeline.py
FRAME_H     = 480   # RGB preview height (pixels) — must match pipeline.py
CAMERA_FPS  = 30    # Frames per second for both RGB and depth streams
                    # RPi 4 tip: if CPU load exceeds 90 %, reduce to 15.

# ── AprilTag ──────────────────────────────────────────────────────────────────
# Print tags from the tag36h11 family at 15 cm × 15 cm for reliable detection
# at 3–4 m.  Tag ID 0 is reserved for the workstation return marker.
APRILTAG_FAMILY        = "tag36h11"
APRILTAG_QUAD_DECIMATE = 3.0    # Downsample factor before quad detection.
                                 # RPi 4: 3.0 recommended (~4× faster vs 1.0).
                                 # RPi 5: 2.0 is fine (faster CPU).
                                 # 15 cm tags detected reliably at 3+ m at 3.0.
WORKSTATION_TAG_ID     = 0      # The AprilTag ID mounted at the home workstation
