"""
OpenCockpit Wireless Bus - PC Bridge Software

Main entry point for the bridge application that connects the
ESP32 coordinator to the flight simulator.
"""

import argparse
import logging
import sys
import time
from typing import Optional

from slip_serial import SlipSerial

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger('bridge')


# Message types (must match firmware/common/protocol.h)
MSG_DISCOVERY_REQ = 0x01
MSG_DISCOVERY_RSP = 0x02
MSG_REGISTER_REQ = 0x03
MSG_REGISTER_ACK = 0x04
MSG_HEARTBEAT = 0x10
MSG_HEARTBEAT_ACK = 0x11
MSG_HID_INPUT = 0x20
MSG_HID_OUTPUT = 0x21
MSG_SERIAL_DATA = 0x30
MSG_MCDU_DISPLAY = 0x40
MSG_MCDU_INPUT = 0x41
MSG_ERROR = 0xF0
MSG_RESET = 0xFF

# Node IDs
NODE_COORDINATOR = 0x00
NODE_B = 0x01  # Joystick + MCDU
NODE_C = 0x02  # Quadrant + FCU
NODE_BROADCAST = 0xFF

# Frame header
FRAME_HEADER = 0xAA


def parse_message(data: bytes) -> Optional[dict]:
    """
    Parse a message from the coordinator.

    Args:
        data: Raw message bytes

    Returns:
        Parsed message dict or None if invalid
    """
    if len(data) < 5:  # Minimum: header + type + src + dst + checksum
        logger.warning(f"Message too short: {len(data)} bytes")
        return None

    if data[0] != FRAME_HEADER:
        logger.warning(f"Invalid header: 0x{data[0]:02X}")
        return None

    msg = {
        'type': data[1],
        'src': data[2],
        'dst': data[3],
        'payload': data[4:-1],
        'checksum': data[-1]
    }

    # TODO: Verify CRC8 checksum

    return msg


def build_message(msg_type: int, src: int, dst: int, payload: bytes = b'') -> bytes:
    """
    Build a message to send to the coordinator.

    Args:
        msg_type: Message type ID
        src: Source node ID
        dst: Destination node ID
        payload: Message payload bytes

    Returns:
        Complete message bytes (without SLIP framing)
    """
    msg = bytearray([FRAME_HEADER, msg_type, src, dst])
    msg.extend(payload)

    # Calculate CRC8 (simple XOR for now - TODO: proper CRC8)
    checksum = 0
    for byte in msg:
        checksum ^= byte
    msg.append(checksum)

    return bytes(msg)


class BridgeApplication:
    """
    Main bridge application class.
    """

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.slip: Optional[SlipSerial] = None
        self.running = False
        self.nodes = {}  # Registered nodes

    def connect(self) -> bool:
        """Connect to the coordinator."""
        try:
            self.slip = SlipSerial(self.port, self.baudrate)
            logger.info(f"Connected to coordinator on {self.port}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False

    def disconnect(self):
        """Disconnect from the coordinator."""
        if self.slip:
            self.slip.close()
            self.slip = None
            logger.info("Disconnected from coordinator")

    def send_discovery(self):
        """Send discovery request to find all nodes."""
        msg = build_message(MSG_DISCOVERY_REQ, NODE_COORDINATOR, NODE_BROADCAST)
        self.slip.send(msg)
        logger.info("Sent discovery request")

    def handle_message(self, msg: dict):
        """
        Handle a received message from the coordinator.

        Args:
            msg: Parsed message dict
        """
        msg_type = msg['type']
        src = msg['src']

        if msg_type == MSG_DISCOVERY_RSP:
            logger.info(f"Discovery response from node {src}")
            self.nodes[src] = {'capabilities': msg['payload']}

        elif msg_type == MSG_HID_INPUT:
            self.handle_hid_input(src, msg['payload'])

        elif msg_type == MSG_SERIAL_DATA:
            self.handle_serial_data(src, msg['payload'])

        elif msg_type == MSG_MCDU_INPUT:
            self.handle_mcdu_input(src, msg['payload'])

        elif msg_type == MSG_HEARTBEAT:
            # Respond with heartbeat ack
            ack = build_message(MSG_HEARTBEAT_ACK, NODE_COORDINATOR, src)
            self.slip.send(ack)

        elif msg_type == MSG_ERROR:
            logger.error(f"Error from node {src}: {msg['payload'].hex()}")

        else:
            logger.debug(f"Unknown message type 0x{msg_type:02X} from node {src}")

    def handle_hid_input(self, src: int, payload: bytes):
        """Handle HID input report from a peripheral node."""
        if len(payload) < 3:
            return

        device_id = payload[0]
        report_id = payload[1]
        report_len = payload[2]
        report_data = payload[3:3 + report_len]

        logger.debug(f"HID input from node {src}, device {device_id}: {report_data.hex()}")

        # TODO: Route to appropriate device handler
        # - Joystick handler for Thrustmaster
        # - Send to vJoy or simulator

    def handle_serial_data(self, src: int, payload: bytes):
        """Handle serial data from a peripheral node (e.g., MiniFCU)."""
        if len(payload) < 2:
            return

        port_id = payload[0]
        data_len = payload[1]
        data = payload[2:2 + data_len]

        logger.debug(f"Serial data from node {src}, port {port_id}: {data}")

        # TODO: Route to FCU handler
        # - Parse MiniFCU events
        # - Send to Fenix API

    def handle_mcdu_input(self, src: int, payload: bytes):
        """Handle MCDU button press from a peripheral node."""
        logger.debug(f"MCDU input from node {src}: {payload.hex()}")

        # TODO: Route to MCDU handler
        # - Parse button index
        # - Send to Fenix MCDU API

    def run(self):
        """Main application loop."""
        if not self.connect():
            return

        self.running = True
        logger.info("Bridge application started")

        # Initial discovery
        self.send_discovery()

        try:
            while self.running:
                # Check for incoming messages
                frame = self.slip.receive()
                if frame:
                    msg = parse_message(frame)
                    if msg:
                        self.handle_message(msg)

                # TODO: Poll simulator state and send updates to peripherals
                # TODO: Periodic heartbeat check

                time.sleep(0.001)  # Prevent busy-waiting

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        finally:
            self.running = False
            self.disconnect()

    def stop(self):
        """Stop the application."""
        self.running = False


def main():
    parser = argparse.ArgumentParser(
        description='OpenCockpit Wireless Bus - PC Bridge Software'
    )
    parser.add_argument(
        '-p', '--port',
        required=True,
        help='Serial port (e.g., COM3 or /dev/ttyUSB0)'
    )
    parser.add_argument(
        '-b', '--baudrate',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )
    parser.add_argument(
        '-v', '--verbose',
        action='store_true',
        help='Enable verbose logging'
    )

    args = parser.parse_args()

    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)

    app = BridgeApplication(args.port, args.baudrate)
    app.run()


if __name__ == '__main__':
    main()
