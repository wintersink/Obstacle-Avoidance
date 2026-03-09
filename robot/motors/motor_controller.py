"""
motors/motor_controller.py  —  Yahboom 4-channel encoder motor driver over I2C.

The Yahboom Quad-MD module contains an STM32F103RCT6 co-processor that
accepts motor commands via I2C.  No GPIO PWM wiring is required — all speed
and direction control goes through the two I2C wires (SDA + SCL).

Hardware setup (Raspberry Pi 4 / 5 — same GPIO pins on both):
    Pin 3  (GPIO 2, SDA)  →  Yahboom SDA
    Pin 5  (GPIO 3, SCL)  →  Yahboom SCL
    Pin 6  (GND)          →  Yahboom GND
    Pin 4  (5V)           →  Yahboom 5V (logic power only)
    External 7.4 V battery →  Yahboom motor power terminals

Enable I2C:  sudo raspi-config → Interface Options → I2C → Enable
Check address: sudo i2cdetect -y 1  (expect 0x34)

─────────────────────────────────────────────────────────────────────────────
IMPORTANT — Yahboom I2C register protocol:
─────────────────────────────────────────────────────────────────────────────
The exact register layout is documented at:
    https://www.yahboom.net/study/Quad-MD-Module

Download the "IIC.py" example from their site and compare it with the
register constants below.  The implementation here follows the most common
Yahboom STM32 motor driver protocol.  If your module responds differently,
adjust _REG_MOTOR_TYPE and _REG_MOTOR_SPEED_BASE accordingly.

Protocol summary (based on Yahboom STM32 driver documentation):
    1. On startup, write the motor type once (one-time configuration).
    2. To set motor speeds, write a signed 16-bit value per channel.
       Positive = forward, negative = reverse, 0 = stop.
       Speed range: -1000 to +1000 (the driver scales to PWM duty cycle).

Register map:
    0x20  — Set motor type (1 byte: MOTOR_TYPE from config.py)
    0x31  — Motor 1 speed (2 bytes, big-endian signed int16)
    0x32  — Motor 2 speed (2 bytes, big-endian signed int16)
    0x33  — Motor 3 speed (2 bytes, big-endian signed int16)
    0x34  — Motor 4 speed (2 bytes, big-endian signed int16)

If the above registers don't work for your specific firmware version,
check the Yahboom documentation for the IIC command set.
"""

from __future__ import annotations

import struct
import time

import smbus2

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    I2C_BUS,
    MOTOR_I2C_ADDR,
    MOTOR_TYPE,
    MOTOR_LEFT_CH,
    MOTOR_RIGHT_CH,
    MOTOR_LEFT_REVERSED,
    MOTOR_RIGHT_REVERSED,
    SPEED_FORWARD,
)

# ── Yahboom register addresses ────────────────────────────────────────────────
# Verify these against https://www.yahboom.net/study/Quad-MD-Module
_REG_MOTOR_TYPE        = 0x20   # Write motor type preset (1 byte)
_REG_MOTOR_SPEED_BASE  = 0x30   # Speed register = base + channel (1-indexed)
                                  # e.g. channel 1 → 0x31, channel 2 → 0x32

# Speed scale sent to the Yahboom driver.
# The driver maps ±_SPEED_SCALE to 0–100 % PWM duty cycle.
_SPEED_SCALE = 1000


class MotorController:
    """
    Controls the two tracks of the Hiwonder tank chassis via the Yahboom
    4-channel encoder motor driver over I2C.

    Public API:
        set_speeds(left, right)  — values in [-100, 100]; positive = forward
        stop()                   — set both tracks to 0
        cleanup()                — stop motors and close I2C bus
    """

    def __init__(self) -> None:
        self._bus = smbus2.SMBus(I2C_BUS)
        self._configure_motor_type()

    def _configure_motor_type(self) -> None:
        """Write the motor type preset to the driver (one-time setup)."""
        try:
            self._bus.write_byte_data(MOTOR_I2C_ADDR, _REG_MOTOR_TYPE, MOTOR_TYPE)
            time.sleep(0.05)  # allow STM32 to process the configuration
        except OSError as exc:
            raise RuntimeError(
                f"Failed to configure Yahboom motor driver at I2C address "
                f"0x{MOTOR_I2C_ADDR:02X} on bus {I2C_BUS}.  "
                f"Is I2C enabled?  Check: sudo i2cdetect -y {I2C_BUS}\n"
                f"Original error: {exc}"
            ) from exc

    def set_speeds(self, left: int, right: int) -> None:
        """
        Set left and right track speeds.

        Args:
            left  — speed in [-100, 100]; positive = forward, negative = reverse
            right — speed in [-100, 100]; positive = forward, negative = reverse

        Values are clamped to [-100, 100] before sending.
        The MOTOR_LEFT_REVERSED / MOTOR_RIGHT_REVERSED flags in config.py
        can invert individual channels if tracks run the wrong way on first use.
        """
        left  = max(-100, min(100, left))
        right = max(-100, min(100, right))

        if MOTOR_LEFT_REVERSED:
            left = -left
        if MOTOR_RIGHT_REVERSED:
            right = -right

        # Scale from [-100, 100] → [-_SPEED_SCALE, _SPEED_SCALE]
        left_raw  = int(left  * _SPEED_SCALE / 100)
        right_raw = int(right * _SPEED_SCALE / 100)

        self._write_speed(MOTOR_LEFT_CH,  left_raw)
        self._write_speed(MOTOR_RIGHT_CH, right_raw)

    def _write_speed(self, channel: int, speed_raw: int) -> None:
        """
        Write a signed 16-bit speed value to the channel register.

        Args:
            channel   — motor channel (1–4)
            speed_raw — signed int in [-_SPEED_SCALE, _SPEED_SCALE]
        """
        reg = _REG_MOTOR_SPEED_BASE + channel
        # Pack as big-endian signed 16-bit integer → 2 bytes
        data = list(struct.pack(">h", speed_raw))
        try:
            self._bus.write_i2c_block_data(MOTOR_I2C_ADDR, reg, data)
        except OSError as exc:
            # Non-fatal: log and continue.  A single missed speed update is
            # preferable to crashing the control loop.
            print(f"[Motor] I2C write error ch={channel} speed={speed_raw}: {exc}")

    def stop(self) -> None:
        """Immediately stop both tracks."""
        self.set_speeds(0, 0)

    def cleanup(self) -> None:
        """Stop motors and release the I2C bus.  Call on shutdown."""
        self.stop()
        try:
            self._bus.close()
        except Exception:
            pass


# ── Standalone test ───────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("Motor controller standalone test")
    print("─" * 40)
    print("CAUTION: Place the robot on a surface where it can move safely.")
    print("The robot will move forward, then rotate, then stop.")
    input("Press Enter to begin …")

    motors = MotorController()

    print(f"Forward at speed {SPEED_FORWARD} for 2 seconds …")
    motors.set_speeds(SPEED_FORWARD, SPEED_FORWARD)
    time.sleep(2.0)

    print("Rotating right for 1 second …")
    motors.set_speeds(SPEED_FORWARD, -SPEED_FORWARD)
    time.sleep(1.0)

    print("Rotating left for 1 second …")
    motors.set_speeds(-SPEED_FORWARD, SPEED_FORWARD)
    time.sleep(1.0)

    print("Stopping.")
    motors.stop()
    motors.cleanup()

    print(
        "\nIf either track ran backward during the forward test, "
        "set MOTOR_LEFT_REVERSED=True or MOTOR_RIGHT_REVERSED=True in config.py."
    )
