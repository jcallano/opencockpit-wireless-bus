"""
Direct USB HID Analyzer for OpenCockpit Optimization
----------------------------------------------------
This script connects directly to Thrustmaster TCA devices via USB (hidapi)
to measure:
1. Polling Frequency (Hz)
2. Jitter (Variance in arrival time)
3. Signal Noise (jitter in axes when idle)

Requirements:
    pip install hidapi

Usage:
    python direct_usb_tool.py
"""

import hid
import time
import statistics
import sys
import collections

# Target Devices
DEVICES = [
    {'vid': 0x044F, 'pid': 0x0405, 'name': 'TCA Sidestick Airbus Edition'},
    {'vid': 0x044F, 'pid': 0x0407, 'name': 'TCA Quadrant Airbus Edition'},
]

class Analyzer:
    def __init__(self):
        self.device = None
        self.running = True
        self.timestamps = collections.deque(maxlen=100)
        self.reports = collections.deque(maxlen=100)
        self.start_time = 0

    def list_devices(self):
        print("Scanning for supported devices...")
        found = []
        for d in hid.enumerate():
            for target in DEVICES:
                if d['vendor_id'] == target['vid'] and d['product_id'] == target['pid']:
                    found.append(d)
        return found

    def analyze(self, dev_info):
        print(f"\nOpening {dev_info['product_string']}...")
        try:
            self.device = hid.device()
            self.device.open_path(dev_info['path'])
            self.device.set_nonblocking(True)
            
            print("Listening for reports... (Press Ctrl+C to stop)")
            print("-" * 60)
            print(f"{'Time (s)':<10} | {'Hz':<10} | {'Jitter (ms)':<12} | {'Bytes':<50}")
            print("-" * 60)

            self.start_time = time.time()
            last_print = 0
            packet_count = 0

            while self.running:
                data = self.device.read(64)
                if data:
                    now = time.time()
                    self.timestamps.append(now)
                    self.reports.append(data)
                    packet_count += 1
                
                # Report stats every second
                if time.time() - last_print > 1.0:
                    self.print_stats()
                    last_print = time.time()
                    packet_count = 0
                
                # Sleep briefly to avoid 100% CPU
                time.sleep(0.0005)

        except KeyboardInterrupt:
            print("\nStopping...")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if self.device:
                self.device.close()

    def print_stats(self):
        if len(self.timestamps) < 2:
            return

        # Calculate Frequency
        duration = self.timestamps[-1] - self.timestamps[0]
        if duration == 0: return
        freq = (len(self.timestamps) - 1) / duration

        # Calculate Jitter (Standard deviation of intervals)
        intervals = [t2 - t1 for t1, t2 in zip(list(self.timestamps)[:-1], list(self.timestamps)[1:])]
        jitter_ms = statistics.stdev(intervals) * 1000 if len(intervals) > 1 else 0

        # Show hex of last report
        last_data = bytes(self.reports[-1]).hex(' ')[:48]
        if len(self.reports[-1]) > 16: last_data += "..."

        print(f"{time.time() - self.start_time:<10.1f} | {freq:<10.2f} | {jitter_ms:<12.3f} | {last_data}")

def main():
    analyzer = Analyzer()
    devices = analyzer.list_devices()

    if not devices:
        print("No supported devices found!")
        print("Plug in your TCA Sidestick or Quadrant.")
        return

    print("\nSelect device to analyze:")
    for i, d in enumerate(devices):
        print(f"[{i}] {d['product_string']} (path: {d['path']})")
    
    try:
        idx = int(input("\nEnter choice [0]: ") or 0)
        target = devices[idx]
        analyzer.analyze(target)
    except ValueError:
        print("Invalid selection.")
    except IndexError:
        print("Invalid selection.")

if __name__ == "__main__":
    main()
