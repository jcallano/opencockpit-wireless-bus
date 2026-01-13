"""
FCU handler for Minicockpit MiniFCU + MiniEFIS.

Translates serial protocol events from the wireless bus
into simulator API calls, and sends display updates back.
"""

import logging
from dataclasses import dataclass, field
from typing import Callable, Optional, Dict

logger = logging.getLogger(__name__)


@dataclass
class FCUState:
    """Current state of the FCU displays and LEDs."""
    # Displays
    speed: int = 0
    heading: int = 0
    altitude: int = 0
    vs_fpa: int = 0
    baro: int = 1013

    # LEDs (True = ON)
    ap1: bool = False
    ap2: bool = False
    athr: bool = False
    loc: bool = False
    exped: bool = False
    appr: bool = False

    # EFIS LEDs
    fd: bool = False
    ls: bool = False
    cstr: bool = False
    wpt: bool = False
    vord: bool = False
    ndb: bool = False
    arpt: bool = False

    # Modes
    speed_managed: bool = False
    heading_managed: bool = False
    altitude_managed: bool = False
    baro_std: bool = False


# Event codes from MiniFCU (Hardware -> PC)
FCU_EVENTS = {
    # Speed encoder
    '11': 'speed_push',
    '12': 'speed_pull',
    '13': 'speed_cw',
    '14': 'speed_ccw',
    # Heading encoder
    '1': 'heading_push',
    '2': 'heading_pull',
    '3': 'heading_cw',
    '4': 'heading_ccw',
    # Altitude encoder
    '15': 'altitude_push',
    '16': 'altitude_pull',
    '17': 'altitude_cw',
    '18': 'altitude_ccw',
    # VS/FPA encoder
    '19': 'vs_push',
    '20': 'vs_pull',
    '21': 'vs_cw',
    '22': 'vs_ccw',
    # BARO encoder
    '69': 'baro_pull',
    '70': 'baro_push',
    # FCU Buttons
    '50': 'ap1',
    '51': 'ap2',
    '52': 'athr',
    '53': 'loc',
    '54': 'exped',
    '55': 'appr',
    '56': 'spd_mach',
    '57': 'hdg_trk',
    '58': 'metric',
    # Altitude increment
    '59': 'alt_100',
    '60': 'alt_1000',
    # EFIS Buttons
    '62': 'fd',
    '63': 'ls',
    '64': 'cstr',
    '65': 'wpt',
    '66': 'vord',
    '67': 'ndb',
    '68': 'arpt',
}


class FCUHandler:
    """
    Handler for Minicockpit MiniFCU + MiniEFIS.

    Parses serial protocol events and generates display/LED commands.
    """

    def __init__(self):
        self.state = FCUState()
        self._event_callbacks: Dict[str, Callable] = {}
        self._pending_commands = []

    def register_event_callback(self, event: str, callback: Callable):
        """
        Register a callback for a specific FCU event.

        Args:
            event: Event name (e.g., 'ap1', 'speed_cw')
            callback: Function to call when event occurs
        """
        self._event_callbacks[event] = callback

    def handle_serial_data(self, data: bytes):
        """
        Process serial data from the MiniFCU.

        Args:
            data: Raw serial bytes (may contain multiple events)
        """
        # Decode ASCII and split by semicolons
        try:
            text = data.decode('ascii')
        except UnicodeDecodeError:
            logger.warning(f"Invalid ASCII in FCU data: {data.hex()}")
            return

        # Split into individual events
        events = text.split(';')
        for event in events:
            event = event.strip()
            if not event:
                continue
            self._parse_event(event)

    def _parse_event(self, event: str):
        """
        Parse a single FCU event.

        Args:
            event: Event string (e.g., '50' for AP1, '13' for speed CW)
        """
        # Check for value events (e.g., '101,1013' for BARO)
        if ',' in event:
            parts = event.split(',')
            code = parts[0]
            value = parts[1] if len(parts) > 1 else None
        else:
            code = event
            value = None

        # Look up event name
        event_name = FCU_EVENTS.get(code)
        if event_name:
            logger.debug(f"FCU event: {event_name}" + (f" = {value}" if value else ""))

            # Call registered callback
            if event_name in self._event_callbacks:
                try:
                    self._event_callbacks[event_name](value)
                except Exception as e:
                    logger.error(f"Event callback error: {e}")
        else:
            logger.debug(f"Unknown FCU event code: {code}")

    def build_display_command(self, display: str, value: int) -> bytes:
        """
        Build a display update command.

        Args:
            display: Display name ('speed', 'heading', 'altitude', 'vs', 'baro')
            value: Value to display

        Returns:
            Command bytes to send to MiniFCU
        """
        commands = {
            'speed': f'S{value:03d},',
            'heading': f'H{value:03d},',
            'altitude': f'A{value:05d},',
            'vs': f'F{value:+05d},',
            'baro': f'#{value:04d},',
        }

        cmd = commands.get(display, '')
        return cmd.encode('ascii')

    def build_led_command(self, led: str, on: bool) -> bytes:
        """
        Build an LED control command.

        Args:
            led: LED name (e.g., 'ap1', 'loc', 'fd')
            on: True for ON, False for OFF

        Returns:
            Command bytes to send to MiniFCU
        """
        # FCU LEDs (uppercase = ON, lowercase = OFF)
        fcu_leds = {
            'ap1': ('P', 'p'),
            'ap2': ('U', 'u'),
            'athr': ('T', 't'),
            'loc': ('L', 'l'),
            'exped': ('E', 'e'),
            'appr': ('R', 'r'),
        }

        # EFIS LEDs (numeric pairs)
        efis_leds = {
            'fd': ('51', '50'),
            'ls': ('41', '40'),
            'cstr': ('31', '30'),
            'wpt': ('21', '20'),
            'vord': ('11', '10'),
            'ndb': ('01', '00'),
            'arpt': ('!1', '!0'),
        }

        if led in fcu_leds:
            on_cmd, off_cmd = fcu_leds[led]
            cmd = (on_cmd if on else off_cmd) + ','
        elif led in efis_leds:
            on_cmd, off_cmd = efis_leds[led]
            cmd = (on_cmd if on else off_cmd) + ','
        else:
            logger.warning(f"Unknown LED: {led}")
            return b''

        return cmd.encode('ascii')

    def update_state(self, **kwargs):
        """
        Update FCU state and generate commands.

        Args:
            **kwargs: State fields to update
        """
        commands = []

        for key, value in kwargs.items():
            if hasattr(self.state, key):
                old_value = getattr(self.state, key)
                if old_value != value:
                    setattr(self.state, key, value)

                    # Generate command based on field type
                    if key in ('speed', 'heading', 'altitude', 'vs_fpa', 'baro'):
                        commands.append(self.build_display_command(
                            key.replace('_fpa', '').replace('vs', 'vs'),
                            value
                        ))
                    elif key in ('ap1', 'ap2', 'athr', 'loc', 'exped', 'appr',
                                 'fd', 'ls', 'cstr', 'wpt', 'vord', 'ndb', 'arpt'):
                        commands.append(self.build_led_command(key, value))

        self._pending_commands.extend(commands)

    def get_pending_commands(self) -> bytes:
        """
        Get all pending commands and clear the queue.

        Returns:
            Concatenated command bytes
        """
        result = b''.join(self._pending_commands)
        self._pending_commands.clear()
        return result
