#!/usr/bin/env python3
"""
OpenCockpit Wireless Bus - Automated Test Runner

Tests:
  - Discovery (nodes register within timeout)
  - Latency (RTT for MSG_TEST)
  - Throughput (messages per second with MSG_TEST)
"""

import argparse
import os
import struct
import sys
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(os.path.join(ROOT, "bridge"))

from slip_serial import SlipSerial  # noqa: E402

FRAME_HEADER = 0xAA

MSG_DISCOVERY_REQ = 0x01
MSG_DISCOVERY_RSP = 0x02
MSG_SERIAL_DATA = 0x30
MSG_TEST_REQ = 0xE0
MSG_TEST_RSP = 0xE1

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


class TestRunner:
    def __init__(self, port: str, baudrate: int, verbose: bool = False):
        if verbose:
            print(f"Opening SLIP port {port} @ {baudrate}")
        self.slip = SlipSerial(port, baudrate)
        if verbose:
            print("SLIP port opened")
        time.sleep(1.0)
        self.verbose = verbose

    def close(self):
        self.slip.close()

    def send(self, msg: bytes):
        if self.verbose:
            print(f"SLIP send {len(msg)} bytes: {msg.hex()}")
        self.slip.send(msg)

    def recv(self, timeout_s: float = 1.0):
        end = time.time() + timeout_s
        while time.time() < end:
            frame = self.slip.receive()
            if frame:
                msg = parse_message(frame)
                if msg:
                    return msg
            time.sleep(0.001)
        return None

    def discovery(self, timeout_s: float, expected_nodes: int):
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
                if self.verbose:
                    print(f"Discovery: node {msg['src']} payload {msg['payload'].hex()}")
            if expected_nodes and len(found) >= expected_nodes:
                break
        return found

    def send_serial(self, node_id: int, port_id: int, data: bytes):
        if len(data) > 200:
            raise ValueError("serial payload too large")
        payload = bytes([port_id, len(data)]) + data
        msg = build_message(MSG_SERIAL_DATA, NODE_COORDINATOR, node_id, payload)
        self.send(msg)

    def latency(self, node_id: int, samples: int, payload_size: int, timeout_s: float):
        rtts = []
        payload_extra = b"\x00" * max(0, payload_size - 8)
        for seq in range(samples):
            ts = int(time.time() * 1000) & 0xFFFFFFFF
            payload = struct.pack("<II", seq, ts) + payload_extra
            msg = build_message(MSG_TEST_REQ, NODE_COORDINATOR, node_id, payload)
            self.send(msg)

            start = time.time()
            while time.time() - start < timeout_s:
                rsp = self.recv(timeout_s=0.01)
                if not rsp:
                    continue
                if rsp["type"] != MSG_TEST_RSP or rsp["src"] != node_id:
                    continue
                if rsp["payload"][:8] != payload[:8]:
                    continue
                rtts.append((time.time() - start) * 1000.0)
                break
        return rtts

    def throughput(self, node_id: int, count: int, payload_size: int, timeout_s: float):
        payload_extra = b"\x00" * max(0, payload_size - 8)
        start = time.time()
        for seq in range(count):
            ts = int(time.time() * 1000) & 0xFFFFFFFF
            payload = struct.pack("<II", seq, ts) + payload_extra
            msg = build_message(MSG_TEST_REQ, NODE_COORDINATOR, node_id, payload)
            self.send(msg)

        received = 0
        while time.time() - start < timeout_s and received < count:
            rsp = self.recv(timeout_s=0.01)
            if rsp and rsp["type"] == MSG_TEST_RSP and rsp["src"] == node_id:
                received += 1

        duration = time.time() - start
        rate = received / duration if duration > 0 else 0
        return received, rate, duration


def percentile(values, pct):
    if not values:
        return None
    values = sorted(values)
    k = int((len(values) - 1) * pct)
    return values[k]


def main():
    parser = argparse.ArgumentParser(description="OpenCockpit Wireless Bus Test Runner")
    parser.add_argument("--port", required=True, help="Serial port (e.g., COM3 or /dev/ttyACM0)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument("--node-id", type=lambda x: int(x, 0), default=0x02, help="Target node ID")
    parser.add_argument("--expected-nodes", type=int, default=2, help="Expected nodes in discovery")
    parser.add_argument("--discovery-timeout", type=float, default=5.0, help="Discovery timeout (s)")
    parser.add_argument("--latency-samples", type=int, default=50, help="Latency samples")
    parser.add_argument("--latency-timeout", type=float, default=0.2, help="Latency response timeout (s)")
    parser.add_argument("--throughput-count", type=int, default=500, help="Throughput message count")
    parser.add_argument("--throughput-timeout", type=float, default=5.0, help="Throughput timeout (s)")
    parser.add_argument("--payload-size", type=int, default=16, help="Test payload size (bytes)")
    parser.add_argument("--minifcu-init", action="store_true",
                        help="Send MiniFCU handshake/init burst before tests")
    parser.add_argument("--verbose", action="store_true", help="Verbose logging")
    args = parser.parse_args()

    runner = TestRunner(args.port, args.baudrate, verbose=args.verbose)
    try:
        if args.minifcu_init:
            print("MiniFCU init...")
            init_burst = (
                "C,"
                "Q400,"
                "K100,"
                "-99,"
                "+10,"
                "n49000,"
                "b100,"
                "[6000,"
                "]-6000,"
                "Z9900,"
                "X-9900,"
                "I,"
                "Y,"
                "W,"
                "O,"
                "{1,"
                "(3248,"
                "}2200,"
                "=1100,"
                "$745,"
                "%0,"
            ).encode("ascii")
            # Send in chunks to fit ESP-NOW serial payload
            for i in range(0, len(init_burst), 180):
                runner.send_serial(args.node_id, 0, init_burst[i:i + 180])
                time.sleep(0.02)
            time.sleep(0.2)

        print("Discovery test...")
        nodes = runner.discovery(args.discovery_timeout, args.expected_nodes)
        print(f"  Found {len(nodes)} node(s): {sorted(nodes.keys())}")

        print("Latency test...")
        rtts = runner.latency(args.node_id, args.latency_samples, args.payload_size, args.latency_timeout)
        if rtts:
            p50 = percentile(rtts, 0.50)
            p90 = percentile(rtts, 0.90)
            p99 = percentile(rtts, 0.99)
            print(f"  RTT ms: p50={p50:.2f} p90={p90:.2f} p99={p99:.2f} samples={len(rtts)}")
        else:
            print("  RTT ms: no responses")

        print("Throughput test...")
        received, rate, duration = runner.throughput(args.node_id, args.throughput_count,
                                                     args.payload_size, args.throughput_timeout)
        print(f"  Received {received}/{args.throughput_count} in {duration:.2f}s -> {rate:.1f} msg/s")
    finally:
        runner.close()


if __name__ == "__main__":
    main()
