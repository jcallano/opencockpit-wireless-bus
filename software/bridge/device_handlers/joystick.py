"""
Joystick handler for Thrustmaster TCA devices.

Translates HID reports from the wireless bus into
joystick inputs for the simulator.
"""

import logging
from dataclasses import dataclass
from typing import Optional

logger = logging.getLogger(__name__)


@dataclass
class SidestickState:
    """Current state of the TCA Sidestick."""
    x_axis: int = 32768  # Center
    y_axis: int = 32768  # Center
    buttons: int = 0
    hat: int = 8  # Center (no direction)


@dataclass
class QuadrantState:
    """Current state of the TCA Quadrant."""
    throttle1: int = 0
    throttle2: int = 0
    flaps: int = 0
    speedbrake: int = 0
    buttons: int = 0


class JoystickHandler:
    """
    Handler for Thrustmaster TCA Sidestick and Quadrant.

    Parses HID reports and provides state access for the simulator bridge.
    """

    # Device IDs (as reported by firmware)
    DEVICE_SIDESTICK = 0x01
    DEVICE_QUADRANT = 0x02

    def __init__(self):
        self.sidestick = SidestickState()
        self.quadrant = QuadrantState()
        self._callbacks = []

    def register_callback(self, callback):
        """Register a callback for state changes."""
        self._callbacks.append(callback)

    def _notify_callbacks(self, device: str, state):
        """Notify all registered callbacks of a state change."""
        for callback in self._callbacks:
            try:
                callback(device, state)
            except Exception as e:
                logger.error(f"Callback error: {e}")

    def handle_hid_report(self, device_id: int, report: bytes):
        """
        Process an HID report from the wireless bus.

        Args:
            device_id: Device identifier
            report: Raw HID report bytes
        """
        if device_id == self.DEVICE_SIDESTICK:
            self._parse_sidestick(report)
        elif device_id == self.DEVICE_QUADRANT:
            self._parse_quadrant(report)
        else:
            logger.warning(f"Unknown device ID: {device_id}")

    def _parse_sidestick(self, report: bytes):
        """
        Parse TCA Sidestick HID report.

        Report format (7 bytes):
        - Bytes 0-1: X axis (16-bit LE)
        - Bytes 2-3: Y axis (16-bit LE)
        - Byte 4: Buttons byte 1
        - Byte 5: Buttons byte 2
        - Byte 6: Hat switch
        """
        if len(report) < 7:
            logger.warning(f"Sidestick report too short: {len(report)}")
            return

        self.sidestick.x_axis = report[0] | (report[1] << 8)
        self.sidestick.y_axis = report[2] | (report[3] << 8)
        self.sidestick.buttons = report[4] | (report[5] << 8)
        self.sidestick.hat = report[6]

        logger.debug(
            f"Sidestick: X={self.sidestick.x_axis}, Y={self.sidestick.y_axis}, "
            f"BTN=0x{self.sidestick.buttons:04X}, HAT={self.sidestick.hat}"
        )

        self._notify_callbacks('sidestick', self.sidestick)

    def _parse_quadrant(self, report: bytes):
        """
        Parse TCA Quadrant HID report.

        Report format varies - see protocol documentation.
        """
        if len(report) < 8:
            logger.warning(f"Quadrant report too short: {len(report)}")
            return

        self.quadrant.throttle1 = report[0] | (report[1] << 8)
        self.quadrant.throttle2 = report[2] | (report[3] << 8)
        self.quadrant.flaps = report[4] | (report[5] << 8)
        self.quadrant.speedbrake = report[6] | (report[7] << 8)

        if len(report) > 8:
            self.quadrant.buttons = report[8]

        logger.debug(
            f"Quadrant: T1={self.quadrant.throttle1}, T2={self.quadrant.throttle2}, "
            f"FLAPS={self.quadrant.flaps}, SPD={self.quadrant.speedbrake}"
        )

        self._notify_callbacks('quadrant', self.quadrant)

    def get_throttle_detent(self, throttle_value: int) -> str:
        """
        Determine throttle detent position.

        Args:
            throttle_value: Raw throttle axis value

        Returns:
            Detent name: 'IDLE', 'CL', 'FLX', 'TOGA', or 'REV'
        """
        if throttle_value < 0x0100:
            return 'IDLE'
        elif throttle_value < 0x4500:
            return 'CL'
        elif throttle_value < 0x6500:
            return 'FLX'
        elif throttle_value < 0x8000:
            return 'TOGA'
        else:
            return 'REV'
