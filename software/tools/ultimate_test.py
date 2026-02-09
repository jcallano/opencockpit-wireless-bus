#!/usr/bin/env python3
"""
Ultimate Test — Full Ursa Minor SLIP Validation Tool (Curses TUI)

Validates all Ursa Minor functionality over SLIP via the coordinator.
Supports HID input decoding, backlight/LED/vibration/LCD output, and discovery.

Usage:
    pip install windows-curses   # Windows only
    python ultimate_test.py --port COM3
"""
import serial
import time
import threading
import struct
import sys
import argparse
import curses
from collections import deque

# ==============================================================================
# CONSTANTS & PROTOCOL
# ==============================================================================
FRAME_HEADER = 0xAA
NODE_COORDINATOR = 0x00
NODE_D_THROTTLE = 0x03
DEV_URSA_MINOR = 0x06

# Message Types
MSG_DISCOVERY_REQ  = 0x01
MSG_DISCOVERY_RSP  = 0x02
MSG_HEARTBEAT      = 0x10
MSG_HID_INPUT      = 0x20
MSG_HID_OUTPUT     = 0x21
MSG_MCDU_DISPLAY   = 0x40
MSG_TEST_RSP       = 0xE1

# SLIP constants
SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD

# All 42 button names (index 0 unused, 1-42 match firmware kBtnNames[])
BTN_NAMES = {
    1:  "ENG 1 MASTER ON",      2:  "ENG 1 MASTER OFF",
    3:  "ENG 2 MASTER ON",      4:  "ENG 2 MASTER OFF",
    5:  "ENG 1 FIRE BUTTON",    6:  "ENG 2 FIRE BUTTON",
    7:  "ENG MODE CRANK",       8:  "ENG MODE NORM",
    9:  "ENG MODE IGN/START",   10: "L THROT AUTO DISC",
    11: "R THROT AUTO DISC",    12: "L THROT TO/GA",
    13: "L THROT FLEX/MCT",     14: "L THROT CL",
    15: "L THROT IDLE",         16: "L THROT IDLE REV",
    17: "L THROT FULL REV",     18: "R THROT TO/GA",
    19: "R THROT FLEX/MCT",     20: "R THROT CL",
    21: "R THROT IDLE",         22: "R THROT IDLE REV",
    23: "R THROT FULL REV",     24: "ENG MODE PUSH BTN",
    25: "TRIM RESET",           26: "RUDDER TRIM L",
    27: "RUDDER TRIM NEUTRAL",  28: "RUDDER TRIM R",
    29: "PARKING BRAKE OFF",    30: "PARKING BRAKE ON",
    31: "FLAPS 4",              32: "FLAPS 3",
    33: "FLAPS 2",              34: "FLAPS 1",
    35: "FLAPS 0",              36: "SPOILER FULL",
    37: "SPOILER ONE HALF",     38: "SPOILER RET",
    39: "SPOILER ARM",          40: "L THROT REV MODE ON",
    41: "R THROT REV MODE ON",  42: "BUTTON 42",
}

# Axis definitions: (byte offset in Report 0x01, name)
AXIS_DEFS = [
    (13, "LEFT THROTTLE"),
    (15, "RIGHT THROTTLE"),
    (19, "SPOILER"),
    (21, "FLAPS"),
]

# Backlight zones: (type, id)
BACKLIGHT_ZONES = [
    (0x10, 0x00),  # Throttle
    (0x10, 0x02),  # Marker
    (0x01, 0x00),  # Flaps
    (0x01, 0x02),  # LCD
]

# ==============================================================================
# CRC8 IMPLEMENTATION (matches firmware)
# ==============================================================================
def crc8(data):
    crc = 0x00
    for byte in data:
        inbyte = byte
        for _ in range(8):
            mix = (crc ^ inbyte) & 0x01
            crc >>= 1
            if mix:
                crc ^= 0x8C
            inbyte >>= 1
    return crc

# ==============================================================================
# SLIP IMPLEMENTATION
# ==============================================================================
def slip_encode(data):
    encoded = bytearray([SLIP_END])
    for byte in data:
        if byte == SLIP_END:
            encoded.extend([SLIP_ESC, SLIP_ESC_END])
        elif byte == SLIP_ESC:
            encoded.extend([SLIP_ESC, SLIP_ESC_ESC])
        else:
            encoded.append(byte)
    encoded.append(SLIP_END)
    return bytes(encoded)


class SlipDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.escaped = False

    def decode_byte(self, byte):
        if self.escaped:
            self.escaped = False
            if byte == SLIP_ESC_END:
                self.buffer.append(SLIP_END)
            elif byte == SLIP_ESC_ESC:
                self.buffer.append(SLIP_ESC)
            else:
                self.buffer.append(byte)
        else:
            if byte == SLIP_END:
                if len(self.buffer) > 0:
                    frame = bytes(self.buffer)
                    self.buffer.clear()
                    return frame
            elif byte == SLIP_ESC:
                self.escaped = True
            else:
                self.buffer.append(byte)
        return None

# ==============================================================================
# MESSAGE BUILDERS
# ==============================================================================
def build_message(msg_type, dst, payload):
    """Build a framed message: AA <type> <src=coordinator> <dst> <payload> <crc8>"""
    header = struct.pack('<BBBB', FRAME_HEADER, msg_type, NODE_COORDINATOR, dst)
    msg = header + payload
    crc = crc8(msg)
    return slip_encode(msg + bytes([crc]))


def build_hid_output(report_id, report_data):
    """Build MSG_HID_OUTPUT with HIDOutputPayload{device_id, report_id, report_length, report_data[64]}"""
    report_bytes = bytes(report_data)
    report_len = len(report_bytes)
    # Pad report_data to 64 bytes
    padded = report_bytes + b'\x00' * (64 - report_len)
    payload = struct.pack('<BBB', DEV_URSA_MINOR, report_id, report_len) + padded
    return build_message(MSG_HID_OUTPUT, NODE_D_THROTTLE, payload)


def build_report02(type_val, id_val, val):
    """Build a Report 0x02 command: [0x02, type, 0xB9, 0x00, 0x00, 0x03, 0x49, id, val, ...zeros]"""
    report = bytearray(64)
    report[0] = 0x02
    report[1] = type_val
    report[2] = 0xB9
    report[3] = 0x00
    report[4] = 0x00
    report[5] = 0x03
    report[6] = 0x49
    report[7] = id_val
    report[8] = val
    return build_hid_output(0x02, report)


def build_backlight_msgs(val):
    """Build 4 Report 0x02 messages to set all backlight zones."""
    return [build_report02(t, i, val) for t, i in BACKLIGHT_ZONES]


def build_led_msg(type_val, id_val, val):
    """Build a single LED/backlight Report 0x02 message."""
    return build_report02(type_val, id_val, val)


def build_vibration_msg(side, val):
    """Build vibration motor command. side='L' -> id=0x0E, 'R' -> id=0x0F."""
    motor_id = 0x0E if side == 'L' else 0x0F
    return build_report02(0x10, motor_id, val)


def build_lcd_msg(side, val_float):
    """Build MSG_MCDU_DISPLAY with text "L 12.3" format.
    Node D parses text as "<L|R> <int>.<frac>" and calls send_lcd()."""
    text = f"{side} {val_float:.1f}"
    text_bytes = text.encode('ascii')
    # MCDUDisplayPayload: command_type(1), row(1), col(1), color(1), length(1), text(24)
    payload = struct.pack('<BBBBB', 0x01, 0, 0, 0, len(text_bytes))
    payload += text_bytes + b'\x00' * (24 - len(text_bytes))
    return build_message(MSG_MCDU_DISPLAY, NODE_D_THROTTLE, payload)


def build_discovery_msg():
    """Build MSG_DISCOVERY_REQ broadcast (dst=0xFF)."""
    return build_message(MSG_DISCOVERY_REQ, 0xFF, b'')

# ==============================================================================
# CURSES TUI APPLICATION
# ==============================================================================

BAR_WIDTH = 12

def axis_bar(val, max_val=65535):
    filled = int(round(val / max_val * BAR_WIDTH))
    filled = max(0, min(BAR_WIDTH, filled))
    return '[' + '=' * filled + ' ' * (BAR_WIDTH - filled) + ']'


def safe_addstr(win, y, x, text, attr=0):
    h, w = win.getmaxyx()
    if y < 0 or y >= h or x >= w:
        return
    max_len = w - x - 1
    if max_len <= 0:
        return
    win.addstr(y, x, text[:max_len], attr)


class UrsaTestTUI:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.decoder = SlipDecoder()
        self.running = True

        # Separate locks: serial_lock for port I/O, state_lock for shared data
        self.serial_lock = threading.Lock()
        self.state_lock = threading.Lock()

        # HID state
        self.axes = [0, 0, 0, 0]
        self.buttons = [False] * 43
        self.last_report = None
        self.hid_ready = False

        # Stats
        self.rx_frames = 0
        self.tx_frames = 0
        self.hid_inputs = 0
        self.heartbeats = 0
        self.node_online = False
        self.start_time = time.time()
        self.last_hid_ts = None
        self.discovery_responses = []

        # Log buffer
        self.log_lines = deque(maxlen=500)

        # Command input buffer (always-on prompt)
        self.input_buf = ""

    def log(self, msg):
        with self.state_lock:
            self.log_lines.append(msg)

    # ------------------------------------------------------------------
    # TX helpers
    # ------------------------------------------------------------------
    def send(self, slip_frame):
        """Send a SLIP frame — dedicated serial lock, flush, error handling."""
        with self.serial_lock:
            try:
                if self.serial and self.serial.is_open:
                    self.serial.write(slip_frame)
                    self.serial.flush()
                    with self.state_lock:
                        self.tx_frames += 1
            except serial.SerialException as e:
                self.log(f"[TX ERROR] {e}")

    def send_multiple(self, frames, delay_ms=50):
        """Send multiple frames with delay. 50ms default gives firmware time to process."""
        for f in frames:
            self.send(f)
            time.sleep(delay_ms / 1000.0)

    # ------------------------------------------------------------------
    # RX thread
    # ------------------------------------------------------------------
    def reader_thread(self):
        while self.running:
            try:
                if self.serial and self.serial.in_waiting:
                    data = self.serial.read(self.serial.in_waiting)
                    for b in data:
                        frame = self.decoder.decode_byte(b)
                        if frame:
                            self.handle_frame(frame)
                else:
                    time.sleep(0.001)
            except serial.SerialException:
                if self.running:
                    self.log("[ERROR] Serial connection lost")
                break

    def handle_frame(self, frame):
        if len(frame) < 5:
            return
        if crc8(frame[:-1]) != frame[-1]:
            return

        with self.state_lock:
            self.rx_frames += 1

        hdr_mark, msg_type, src, dst = struct.unpack('<BBBB', frame[:4])
        payload = frame[4:-1]

        if msg_type == MSG_HID_INPUT:
            self.parse_hid_input(payload)
        elif msg_type == MSG_HEARTBEAT:
            with self.state_lock:
                self.heartbeats += 1
                self.node_online = True
        elif msg_type == MSG_DISCOVERY_RSP:
            self.handle_discovery_response(src, payload)
        elif msg_type == MSG_TEST_RSP:
            self.log(f"[PING] from Node {src}")

    def parse_hid_input(self, payload):
        if len(payload) < 3:
            return
        report_id = payload[1]
        report_data = payload[3:]

        if report_id != 0x01 or len(report_data) < 23:
            return

        with self.state_lock:
            self.hid_inputs += 1
            self.last_hid_ts = time.time()
            self.node_online = True

        cur = bytes(report_data[:64])

        if self.last_report is None:
            self.last_report = cur
            self.hid_ready = True
            with self.state_lock:
                for i, (offset, name) in enumerate(AXIS_DEFS):
                    if offset + 1 < len(cur):
                        self.axes[i] = cur[offset] | (cur[offset + 1] << 8)
                for i in range(6):
                    for b in range(8):
                        btn = i * 8 + b + 1
                        if btn > 42:
                            continue
                        if 1 + i < len(cur):
                            self.buttons[btn] = bool(cur[1 + i] & (1 << b))
            active = [BTN_NAMES.get(n, f"BTN_{n}") for n in range(1, 43) if self.buttons[n]]
            self.log(f"[HID] First report — {len(active)} buttons active")
            return

        prev = self.last_report
        if cur == prev:
            return

        with self.state_lock:
            # Log button changes
            for i in range(6):
                diff = cur[1 + i] ^ prev[1 + i]
                if not diff:
                    continue
                for b in range(8):
                    if not (diff & (1 << b)):
                        continue
                    btn = i * 8 + b + 1
                    if btn > 42:
                        continue
                    pressed = bool(cur[1 + i] & (1 << b))
                    name = BTN_NAMES.get(btn, f"BTN_{btn}")
                    state = "PRESS" if pressed else "REL"
                    self.log_lines.append(f"  BTN {btn:02d} [{name}]: {state}")

            # Update full button state
            for i in range(6):
                for b in range(8):
                    btn = i * 8 + b + 1
                    if btn > 42:
                        continue
                    if 1 + i < len(cur):
                        self.buttons[btn] = bool(cur[1 + i] & (1 << b))

            # Update axes
            for idx, (offset, name) in enumerate(AXIS_DEFS):
                if offset + 1 < len(cur):
                    self.axes[idx] = cur[offset] | (cur[offset + 1] << 8)

        self.last_report = cur

    def handle_discovery_response(self, src, payload):
        ts = time.time() - self.start_time
        with self.state_lock:
            self.discovery_responses.append((src, ts))
            self.node_online = True
        self.log(f"[DISCOVERY] Node {src} responded ({ts:.1f}s)")

    # ------------------------------------------------------------------
    # Command processor — same commands as the old tool
    # ------------------------------------------------------------------
    @staticmethod
    def parse_int(s):
        try:
            return int(s, 0)
        except ValueError:
            return None

    def process_command(self, line):
        """Process a command line. Returns False to quit."""
        parts = line.strip().split()
        if not parts:
            return True

        cmd = parts[0].upper()

        if cmd in ('QUIT', 'Q', 'EXIT'):
            return False

        elif cmd in ('HELP', 'H', '?'):
            self.log("=== Commands ===")
            self.log("  BL <0-255>              All 4 backlight zones")
            self.log("  LED <type> <id> <val>   Single Report 0x02 (hex ok)")
            self.log("  VIB <L|R> <0-255>       Vibration motor")
            self.log("  LCD <L|R> <0.0-99.9>    7-segment display")
            self.log("  DISC                    Trigger discovery")
            self.log("  STATUS                  Connection stats")
            self.log("  RAW <hex>               Send raw hex as SLIP")
            self.log("  QUIT                    Exit")

        elif cmd == 'BL':
            if len(parts) < 2:
                self.log("Usage: BL <0-255>")
                return True
            val = self.parse_int(parts[1])
            if val is None or val < 0 or val > 255:
                self.log("Value must be 0-255")
                return True
            msgs = build_backlight_msgs(val)
            self.send_multiple(msgs)
            self.log(f"BL -> all zones set to {val}")

        elif cmd == 'LED':
            if len(parts) < 4:
                self.log("Usage: LED <type> <id> <val>  (hex ok: 0x10)")
                return True
            type_val = self.parse_int(parts[1])
            id_val = self.parse_int(parts[2])
            val = self.parse_int(parts[3])
            if None in (type_val, id_val, val):
                self.log("Invalid number")
                return True
            self.send(build_led_msg(type_val, id_val, val))
            self.log(f"LED -> type=0x{type_val:02X} id=0x{id_val:02X} val={val}")

        elif cmd == 'VIB':
            if len(parts) < 3:
                self.log("Usage: VIB <L|R> <0-255>")
                return True
            side = parts[1].upper()
            if side not in ('L', 'R'):
                self.log("Side must be L or R")
                return True
            val = self.parse_int(parts[2])
            if val is None or val < 0 or val > 255:
                self.log("Value must be 0-255")
                return True
            self.send(build_vibration_msg(side, val))
            self.log(f"VIB -> {side} motor = {val}")

        elif cmd == 'LCD':
            if len(parts) < 3:
                self.log("Usage: LCD <L|R> <0.0-99.9>")
                return True
            side = parts[1].upper()
            if side not in ('L', 'R'):
                self.log("Side must be L or R")
                return True
            try:
                val = float(parts[2])
            except ValueError:
                self.log("Invalid number")
                return True
            if val < 0.0 or val > 99.9:
                self.log("Value must be 0.0-99.9")
                return True
            self.send(build_lcd_msg(side, val))
            self.log(f"LCD -> {side} = {val:.1f}")

        elif cmd in ('DISC', 'DISCOVERY'):
            self.send(build_discovery_msg())
            self.log("Discovery request sent (broadcast)")

        elif cmd == 'STATUS':
            uptime = time.time() - self.start_time
            with self.state_lock:
                last_hid = "never"
                if self.last_hid_ts is not None:
                    ago = time.time() - self.last_hid_ts
                    last_hid = f"{ago:.1f}s ago"
                disc_nodes = [str(src) for src, _ in self.discovery_responses]
            self.log(f"  Uptime: {uptime:.0f}s")
            self.log(f"  RX: {self.rx_frames}  TX: {self.tx_frames}")
            self.log(f"  HID inputs: {self.hid_inputs}  Last: {last_hid}")
            self.log(f"  Heartbeats: {self.heartbeats}")
            self.log(f"  Discovered: {', '.join(disc_nodes) or 'none'}")

        elif cmd == 'RAW':
            if len(parts) < 2:
                self.log("Usage: RAW <hex bytes>")
                return True
            try:
                raw = bytes.fromhex(''.join(parts[1:]))
                if self.serial and self.serial.is_open:
                    self.serial.write(slip_encode(raw))
                    with self.state_lock:
                        self.tx_frames += 1
                    self.log(f"RAW -> {len(raw)} bytes")
            except ValueError:
                self.log("Invalid hex")

        else:
            self.log(f"Unknown command: {cmd}. Type HELP.")

        return True

    # ------------------------------------------------------------------
    # Drawing
    # ------------------------------------------------------------------
    def draw(self, stdscr):
        h, w = stdscr.getmaxyx()
        if h < 10 or w < 40:
            stdscr.erase()
            safe_addstr(stdscr, 0, 0, "Terminal too small! Need 40x10+")
            stdscr.noutrefresh()
            curses.doupdate()
            return

        stdscr.erase()
        div_col = 34

        # --- Row 0: Title bar ---
        with self.state_lock:
            online_str = "ONLINE" if self.node_online else "OFFLINE"
            rx, tx, hb, hid = self.rx_frames, self.tx_frames, self.heartbeats, self.hid_inputs
        title = " URSA MINOR TEST TOOL"
        stats_r = f"Node D: {online_str}  RX:{rx} TX:{tx} HB:{hb} HID:{hid} "
        pad = w - len(title) - len(stats_r) - 4
        if pad < 0:
            pad = 0
        safe_addstr(stdscr, 0, 0, ('+=' + title + '=' * pad + ' ' + stats_r + '=+')[:w-1], curses.A_BOLD)

        # --- Adaptive layout ---
        # We need: title(1) + header(1) + panel(>=4) + sep(1) + log(>=3) + prompt_border(1) + prompt(1) = min 12
        # panel gets up to 10 rows, log gets the rest
        reserved = 5  # title + header + sep + prompt_border + prompt_line
        avail = h - reserved
        panel_rows = max(4, min(10, (avail * 2) // 3))
        log_rows = max(3, avail - panel_rows)

        # --- Row 1: Column headers ---
        safe_addstr(stdscr, 1, 0, '|')
        safe_addstr(stdscr, 1, 2, 'AXES', curses.A_BOLD)
        safe_addstr(stdscr, 1, div_col, '|')
        safe_addstr(stdscr, 1, div_col + 2, 'ACTIVE BUTTONS', curses.A_BOLD)
        if w > div_col + 20:
            safe_addstr(stdscr, 1, w - 2, '|')

        # --- Rows 2..2+panel_rows: Axes + Buttons ---
        axis_labels = ["L Throttle", "R Throttle", "Spoiler   ", "Flaps     "]
        with self.state_lock:
            axes_snap = list(self.axes)
            btns_snap = list(self.buttons)

        active_btns = []
        for btn_id in range(1, 43):
            if btns_snap[btn_id]:
                active_btns.append(BTN_NAMES.get(btn_id, f"BTN_{btn_id}"))

        for ri in range(panel_rows):
            y = 2 + ri
            safe_addstr(stdscr, y, 0, '|')
            if ri < len(axis_labels):
                val = axes_snap[ri]
                safe_addstr(stdscr, y, 1, f" {axis_labels[ri]}: {val:5d}  {axis_bar(val)}")
            elif ri == len(axis_labels) and not self.hid_ready:
                safe_addstr(stdscr, y, 2, "(waiting for HID data...)")
            safe_addstr(stdscr, y, div_col, '|')
            if ri < len(active_btns):
                safe_addstr(stdscr, y, div_col + 2, active_btns[ri])
            elif ri == 0 and not active_btns:
                safe_addstr(stdscr, y, div_col + 2, "(waiting...)" if not self.hid_ready else "(none)")
            if w > div_col + 2:
                safe_addstr(stdscr, y, w - 2, '|')

        if len(active_btns) > panel_rows:
            safe_addstr(stdscr, 2 + panel_rows - 1, div_col + 2,
                        f"+{len(active_btns) - panel_rows + 1} more...")

        # --- Separator ---
        sep_row = 2 + panel_rows
        sep = '+' + '-' * (div_col - 1) + '+' + '-' * max(0, w - div_col - 3) + '+'
        safe_addstr(stdscr, sep_row, 0, sep[:w-1])

        # --- Log area ---
        log_start = sep_row + 1
        with self.state_lock:
            log_snap = list(self.log_lines)
        visible = log_snap[-log_rows:] if len(log_snap) > log_rows else log_snap

        for i in range(log_rows):
            y = log_start + i
            if y >= h - 2:
                break
            safe_addstr(stdscr, y, 0, '|')
            if i < len(visible):
                safe_addstr(stdscr, y, 2, visible[i])
            if w > 2:
                safe_addstr(stdscr, y, w - 2, '|')

        # --- Prompt border ---
        prompt_border_row = log_start + log_rows
        if prompt_border_row < h - 1:
            border = '+' + '=' * max(0, w - 3) + '+'
            safe_addstr(stdscr, prompt_border_row, 0, border[:w-1])

        # --- Prompt line (bottom) ---
        prompt_row = h - 1
        prompt_str = f"> {self.input_buf}"
        safe_addstr(stdscr, prompt_row, 0, prompt_str)
        # Position cursor after typed text
        cx = 2 + len(self.input_buf)
        if cx < w - 1:
            try:
                stdscr.move(prompt_row, cx)
            except curses.error:
                pass

        stdscr.noutrefresh()
        curses.doupdate()

    # ------------------------------------------------------------------
    # Key handling — persistent "> " prompt, type commands like before
    # ------------------------------------------------------------------
    def handle_key(self, key):
        """Handle a keypress. Returns False to quit."""
        if key in (10, 13, curses.KEY_ENTER):
            # Submit command
            line = self.input_buf
            self.input_buf = ""
            if line.strip():
                self.log(f"> {line}")
                return self.process_command(line)
            return True
        elif key in (curses.KEY_BACKSPACE, 127, 8):
            if self.input_buf:
                self.input_buf = self.input_buf[:-1]
        elif key == 27:  # ESC — clear input
            self.input_buf = ""
        elif key == 21:  # Ctrl+U — clear line
            self.input_buf = ""
        elif 32 <= key <= 126:
            self.input_buf += chr(key)
        return True

    # ------------------------------------------------------------------
    # Main curses loop
    # ------------------------------------------------------------------
    def curses_main(self, stdscr):
        # Show cursor (user types commands)
        try:
            curses.curs_set(1)
        except curses.error:
            pass
        stdscr.nodelay(True)
        stdscr.timeout(100)

        # Open serial
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1, write_timeout=0.5)
            self.serial.dtr = True
            self.serial.rts = True
        except serial.SerialException as e:
            stdscr.nodelay(False)
            stdscr.clear()
            stdscr.addstr(0, 0, f"Failed to open {self.port}: {e}")
            stdscr.addstr(1, 0, "Press any key to exit.")
            stdscr.getch()
            return

        self.log(f"Opened {self.port} @ {self.baudrate}")
        self.log("Type HELP for commands. Type QUIT to exit.")

        # Start reader thread
        reader = threading.Thread(target=self.reader_thread, daemon=True)
        reader.start()

        # Auto-discovery after settle
        time.sleep(1.0)
        if self.serial.in_waiting:
            self.serial.read(self.serial.in_waiting)
        self.send(build_discovery_msg())
        self.log("Discovery request sent (startup)")

        try:
            while self.running:
                key = stdscr.getch()
                if key != -1:
                    if not self.handle_key(key):
                        break
                self.draw(stdscr)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.serial and self.serial.is_open:
                self.serial.close()

    def run(self):
        curses.wrapper(self.curses_main)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Ursa Minor SLIP Validation Tool - Curses TUI via coordinator"
    )
    parser.add_argument("--port", required=True, help="Serial port (e.g. COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default 115200)")
    args = parser.parse_args()

    app = UrsaTestTUI(args.port, args.baud)
    app.run()
