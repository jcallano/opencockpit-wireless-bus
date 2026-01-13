"""
MCDU handler for WinWing MCDU.

Translates button presses from the wireless bus into
simulator API calls, and sends display updates back.
"""

import logging
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class MCDUDisplay:
    """MCDU display buffer (24 columns x 14 rows)."""
    rows: int = 14
    cols: int = 24

    def __post_init__(self):
        # Each cell: (character, color)
        self.buffer: List[List[tuple]] = [
            [(' ', 0) for _ in range(self.cols)]
            for _ in range(self.rows)
        ]

    def set_text(self, row: int, col: int, text: str, color: int = 0):
        """Set text at a specific position."""
        if 0 <= row < self.rows:
            for i, char in enumerate(text):
                if col + i < self.cols:
                    self.buffer[row][col + i] = (char, color)

    def clear(self):
        """Clear the display."""
        for row in range(self.rows):
            for col in range(self.cols):
                self.buffer[row][col] = (' ', 0)

    def get_row(self, row: int) -> str:
        """Get text content of a row."""
        if 0 <= row < self.rows:
            return ''.join(cell[0] for cell in self.buffer[row])
        return ''


# MCDU button indices (from HID report)
MCDU_BUTTONS = {
    # LSK Left (0-5)
    0: 'LSK1L', 1: 'LSK2L', 2: 'LSK3L', 3: 'LSK4L', 4: 'LSK5L', 5: 'LSK6L',
    # LSK Right (6-11)
    6: 'LSK1R', 7: 'LSK2R', 8: 'LSK3R', 9: 'LSK4R', 10: 'LSK5R', 11: 'LSK6R',
    # Function row (12-17)
    12: 'DIR', 13: 'PROG', 14: 'PERF', 15: 'INIT', 16: 'DATA', 17: 'F_PLN',
    # Page controls (18-25)
    18: 'RAD_NAV', 19: 'FUEL_PRED', 20: 'SEC_F_PLN', 21: 'ATC_COMM',
    22: 'MCDU_MENU', 23: 'AIRPORT', 24: 'UP', 25: 'DOWN',
    # Arrow keys + OVFY (26-31)
    26: 'LEFT', 27: 'RIGHT', 28: 'UP_ARROW', 29: 'DOWN_ARROW',
    30: 'OVFY', 31: 'BRT',
    # Number pad (32-41)
    32: '1', 33: '2', 34: '3', 35: '4', 36: '5',
    37: '6', 38: '7', 39: '8', 40: '9', 41: '0',
    # Letter keys A-Z (42-67)
    **{42 + i: chr(65 + i) for i in range(26)},
    # Special keys (68-71)
    68: 'CLR', 69: 'SP', 70: 'SLASH', 71: 'PLUSMINUS',
}


class MCDUHandler:
    """
    Handler for WinWing MCDU.

    Manages display buffer and button events.
    """

    # Display colors
    COLOR_WHITE = 0
    COLOR_GREEN = 1
    COLOR_CYAN = 2
    COLOR_MAGENTA = 3
    COLOR_AMBER = 4
    COLOR_RED = 5

    def __init__(self):
        self.display = MCDUDisplay()
        self._button_callbacks: Dict[str, Callable] = {}
        self._pending_display_updates = []

    def register_button_callback(self, button: str, callback: Callable):
        """
        Register a callback for a specific button.

        Args:
            button: Button name (e.g., 'LSK1L', 'A', '1')
            callback: Function to call when button is pressed
        """
        self._button_callbacks[button] = callback

    def register_all_buttons_callback(self, callback: Callable):
        """
        Register a callback for all button presses.

        Args:
            callback: Function(button_name) to call
        """
        self._all_buttons_callback = callback

    def handle_button_report(self, report: bytes):
        """
        Process a button HID report from the MCDU.

        Args:
            report: 12-byte button state report
        """
        if len(report) < 12:
            logger.warning(f"MCDU button report too short: {len(report)}")
            return

        # Each bit represents a button state
        for byte_idx, byte in enumerate(report):
            for bit_idx in range(8):
                button_idx = byte_idx * 8 + bit_idx
                if byte & (1 << bit_idx):
                    button_name = MCDU_BUTTONS.get(button_idx)
                    if button_name:
                        self._handle_button_press(button_name)

    def _handle_button_press(self, button: str):
        """Handle a single button press."""
        logger.debug(f"MCDU button: {button}")

        # Call specific callback
        if button in self._button_callbacks:
            try:
                self._button_callbacks[button]()
            except Exception as e:
                logger.error(f"Button callback error: {e}")

        # Call all-buttons callback
        if hasattr(self, '_all_buttons_callback'):
            try:
                self._all_buttons_callback(button)
            except Exception as e:
                logger.error(f"All-buttons callback error: {e}")

    def update_display_line(self, row: int, text: str, color: int = 0):
        """
        Update a display line.

        Args:
            row: Row number (0-13)
            text: Text to display (up to 24 chars)
            color: Text color
        """
        self.display.set_text(row, 0, text.ljust(24)[:24], color)
        self._pending_display_updates.append((row, text, color))

    def build_display_command(self, row: int, col: int, text: str, color: int) -> bytes:
        """
        Build a display update command for the MCDU.

        Args:
            row: Row number (0-13)
            col: Column number (0-23)
            text: Text to write
            color: Text color

        Returns:
            HID output report bytes
        """
        # WinWing MCDU protocol
        # Header: 0xF0 0x00 LEN CMD
        # For text: CMD = row, then position + color + text

        cmd = bytearray([0xF0, 0x00])

        # Payload: row, col, color, text...
        payload = bytearray([row, col, color])
        payload.extend(text.encode('ascii', errors='replace'))

        cmd.append(len(payload) + 1)  # Length
        cmd.append(0x01)  # Text command
        cmd.extend(payload)

        # Pad to 64 bytes (HID report size)
        while len(cmd) < 64:
            cmd.append(0x00)

        return bytes(cmd)

    def get_pending_updates(self) -> List[bytes]:
        """
        Get all pending display update commands.

        Returns:
            List of HID output report bytes
        """
        commands = []
        for row, text, color in self._pending_display_updates:
            commands.append(self.build_display_command(row, 0, text, color))

        self._pending_display_updates.clear()
        return commands

    def clear_display(self):
        """Clear the MCDU display."""
        self.display.clear()
        for row in range(14):
            self._pending_display_updates.append((row, ' ' * 24, 0))
