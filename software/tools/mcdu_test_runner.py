#!/usr/bin/env python3
"""
OpenCockpit Wireless Bus - MCDU Test Runner

Tests:
  - Display update via MSG_MCDU_DISPLAY
  - Button events via MSG_MCDU_INPUT
"""

import argparse
import os
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(os.path.join(ROOT, "bridge"))
sys.path.append(os.path.join(ROOT, "bridge", "device_handlers"))

from slip_serial import SlipSerial  # noqa: E402

try:
    from mcdu import MCDU_BUTTONS  # noqa: E402
except Exception:
    MCDU_BUTTONS = {}

FRAME_HEADER = 0xAA

MSG_DISCOVERY_REQ = 0x01
MSG_DISCOVERY_RSP = 0x02
MSG_MCDU_DISPLAY = 0x40
MSG_MCDU_INPUT = 0x41

NODE_COORDINATOR = 0x00
NODE_BROADCAST = 0xFF


def crc8(data: bytes) -> int:
    crc = 0x00
    for inbyte in data:
        for _ in range(8):
            mix = (crc ^ inbyte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            inbyte >>= 1
    return crc & 0xFF


def build_message(msg_type: int, src: int, dst: int, payload: bytes = b"") -> bytes:
    frame = bytearray([FRAME_HEADER, msg_type, src, dst])
    frame.extend(payload)
    frame.append(crc8(frame))
    return bytes(frame)


def parse_message(frame: bytes):
    if len(frame) < 5:
        return None
    if frame[0] != FRAME_HEADER:
        return None
    if crc8(frame[:-1]) != frame[-1]:
        return None
    return {
        "type": frame[1],
        "src": frame[2],
        "dst": frame[3],
        "payload": frame[4:-1],
    }


def build_mcdu_display(row: int, col: int, text: str, color: int) -> bytes:
    text = text[:24]
    payload = bytearray(29)
    payload[0] = 0x01
    payload[1] = row & 0xFF
    payload[2] = col & 0xFF
    payload[3] = color & 0xFF
    payload[4] = len(text) & 0xFF
    payload[5:5 + len(text)] = text.encode("ascii", errors="replace")
    return bytes(payload)


def build_mcdu_brightness() -> bytes:
    payload = bytearray(29)
    payload[0] = 0x03
    return bytes(payload)


class MCDUTestRunner:
    def __init__(self, port: str, baudrate: int = 115200, verbose: bool = False):
        if verbose:
            print(f"Opening SLIP port {port} @ {baudrate}")
        self.slip = SlipSerial(port, baudrate)
        if verbose:
            print("SLIP port opened")
        time.sleep(0.5)
        self.verbose = verbose

    def close(self):
        self.slip.close()

    def send(self, msg: bytes):
        if self.verbose:
            print(f"SLIP send {len(msg)} bytes: {msg.hex()}")
        self.slip.send(msg)

    def recv(self, timeout_s: float = 0.2):
        end = time.time() + timeout_s
        while time.time() < end:
            frame = self.slip.receive()
            if frame:
                msg = parse_message(frame)
                if msg:
                    return msg
            time.sleep(0.001)
        return None

    def discovery(self, timeout_s: float):
        msg = build_message(MSG_DISCOVERY_REQ, NODE_COORDINATOR, NODE_BROADCAST)
        self.send(msg)
        found = {}
        end = time.time() + timeout_s
        while time.time() < end:
            msg = self.recv(timeout_s=0.05)
            if not msg:
                continue
            if msg["type"] == MSG_DISCOVERY_RSP:
                found[msg["src"]] = msg["payload"]
        return found

    def send_display(self, node_id: int, row: int, col: int, text: str, color: int):
        payload = build_mcdu_display(row, col, text, color)
        msg = build_message(MSG_MCDU_DISPLAY, NODE_COORDINATOR, node_id, payload)
        self.send(msg)

    def send_brightness(self, node_id: int):
        payload = build_mcdu_brightness()
        msg = build_message(MSG_MCDU_DISPLAY, NODE_COORDINATOR, node_id, payload)
        self.send(msg)

    def listen_buttons(self, duration_s: float):
        end = time.time() + duration_s
        while time.time() < end:
            msg = self.recv(timeout_s=0.05)
            if not msg:
                continue
            if msg["type"] != MSG_MCDU_INPUT:
                continue
            payload = msg["payload"]
            if len(payload) < 2:
                continue
            button_id = payload[0]
            state = payload[1]
            name = MCDU_BUTTONS.get(button_id, f"BTN_{button_id}")
            print(f"Button {name} ({button_id}) state={state}")


def main():
    parser = argparse.ArgumentParser(description="MCDU display/button test runner")
    parser.add_argument("--port", required=True, help="Serial port (e.g., COM3 or /dev/ttyACM0)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--node-id", type=lambda x: int(x, 0), default=0x01, help="Target node ID")
    parser.add_argument("--duration", type=float, default=10.0, help="Button listen time (s)")
    parser.add_argument("--no-display", action="store_true", help="Skip display updates")
    parser.add_argument("--verbose", action="store_true", help="Verbose logging")
    args = parser.parse_args()

    runner = MCDUTestRunner(args.port, args.baudrate, verbose=args.verbose)
    try:
        if not args.no_display:
            runner.send_display(args.node_id, 0, 0, "OPENCOCKPIT MCDU", 2)
            runner.send_display(args.node_id, 1, 0, "BUTTON TEST READY", 1)
            runner.send_brightness(args.node_id)

        print("Listening for MCDU button events...")
        runner.listen_buttons(args.duration)
    finally:
        runner.close()


if __name__ == "__main__":
    main()
