"""
Frequency Analysis Tool for OpenCockpit Wireless Bus
----------------------------------------------------
Connects to the Coordinator via USB Serial (SLIP) and measures
the frequency and jitter of incoming HID reports from peripheral nodes.

Usage:
    python measure_frequency.py <COM_PORT>
"""

import sys
import os
import time
import struct
import argparse
import logging
from collections import defaultdict, deque
import statistics

# Add bridge directory to path to import slip_serial
sys.path.append(os.path.join(os.path.dirname(__file__), '../bridge'))

try:
    from slip_serial import SlipSerial
except ImportError:
    print("Error: Could not import slip_serial. Make sure software/bridge/slip_serial.py exists.")
    sys.exit(1)

# Message Constants (must match firmware)
FRAME_HEADER = 0xAA
MSG_HID_INPUT = 0x20

# Configure Logging
logging.basicConfig(level=logging.INFO, format='%(message)s')
logger = logging.getLogger('analyzer')

class FrequencyAnalyzer:
    def __init__(self, port):
        self.port = port
        self.running = True
        self.stats = defaultdict(lambda: {
            'count': 0,
            'last_ts': 0.0,
            'intervals': deque(maxlen=100),  # Rolling window for jitter
            'total_packets': 0
        })
        self.start_time = time.time()
        self.last_report_time = time.time()

    def parse_header(self, data):
        if len(data) < 5 or data[0] != FRAME_HEADER:
            return None
        return {
            'type': data[1],
            'src': data[2],
            'dst': data[3],
            'payload': data[4:-1]
        }

    def run(self):
        try:
            with SlipSerial(self.port) as slip:
                print(f"Connected to {self.port}. Listening for HID traffic...")
                print("-" * 60)
                print(f"{'Node':<6} | {'Device':<8} | {'Freq (Hz)':<10} | {'Jitter (ms)':<12} | {'Pkts/s':<8}")
                print("-" * 60)

                while self.running:
                    # Non-blocking receive
                    frame = slip.receive()
                    if frame:
                        self.process_frame(frame)
                    
                    # Report every 1.0s
                    if time.time() - self.last_report_time >= 1.0:
                        self.print_stats()
                        self.last_report_time = time.time()

        except KeyboardInterrupt:
            print("\nStopping analysis...")
        except Exception as e:
            print(f"\nError: {e}")

    def process_frame(self, frame):
        msg = self.parse_header(frame)
        if not msg:
            return

        if msg['type'] == MSG_HID_INPUT:
            payload = msg['payload']
            if len(payload) >= 3:
                src_node = msg['src']
                device_id = payload[0]
                self.record_metric(src_node, device_id)

    def record_metric(self, node_id, device_id):
        key = (node_id, device_id)
        now = time.time()
        stat = self.stats[key]

        if stat['last_ts'] > 0:
            interval = (now - stat['last_ts']) * 1000.0 # ms
            stat['intervals'].append(interval)
        
        stat['last_ts'] = now
        stat['count'] += 1
        stat['total_packets'] += 1

    def print_stats(self):
        # Move cursor up to overwrite previous stats if needed, or just print new lines
        # For simplicity in this CLI, we just print new block or clear screen
        # Using ANSI escape to clear lines would be fancy, but simple print is safer
        
        now = time.time()
        
        active_devices = [k for k, v in self.stats.items() if v['count'] > 0]
        if not active_devices:
            return

        print(f"\n--- Stats @ {now - self.start_time:.1f}s ---")
        print(f"{'Node':<6} | {'Device':<8} | {'Freq (Hz)':<10} | {'Jitter (ms)':<12} | {'Total':<8}")
        
        for key in sorted(active_devices):
            node, device = key
            stat = self.stats[key]
            
            freq = stat['count'] # Since we report every 1.0s, count is freq
            
            jitter = 0.0
            if len(stat['intervals']) > 1:
                jitter = statistics.stdev(stat['intervals'])
            
            print(f"0x{node:02X}   | 0x{device:02X}     | {freq:<10} | {jitter:<12.2f} | {stat['total_packets']:<8}")
            
            # Reset counter for next second
            stat['count'] = 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analyze Wireless Bus Frequency')
    parser.add_argument('port', help='Serial port (e.g., COM3)')
    args = parser.parse_args()

    analyzer = FrequencyAnalyzer(args.port)
    analyzer.run()
