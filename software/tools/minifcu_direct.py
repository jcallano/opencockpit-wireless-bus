#!/usr/bin/env python3
import argparse
import threading
import time

import serial


INIT_BURST = [
    "Q400,",
    "K100,",
    "-99,",
    "+10,",
    "n49000,",
    "b100,",
    "[6000,",
    "]-6000,",
    "Z9900,",
    "X-9900,",
    "I,",
    "Y,",
    "W,",
    "O,",
    "{1,",
    "(3248,",
    "}2200,",
    "=1100,",
    "$745,",
    "%0,",
]


def reader_thread(ser: serial.Serial, stop_event: threading.Event) -> None:
    buffer = ""
    while not stop_event.is_set():
        try:
            data = ser.read(64)
            if not data:
                continue
            text = data.decode("ascii", errors="ignore")
            buffer += text
            while ";" in buffer:
                frame, buffer = buffer.split(";", 1)
                frame = frame.strip()
                if frame:
                    ts = time.strftime("%H:%M:%S")
                    print(f"[RX {ts}] {frame};")
        except Exception as exc:
            print(f"[RX] error: {exc}")
            break


def send_tokens(ser: serial.Serial, tokens) -> None:
    payload = "".join(tokens).encode("ascii")
    ser.write(payload)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Direct MiniFCU serial opener + init + display test"
    )
    parser.add_argument("--port", required=True, help="Serial port (COMx or /dev/ttyUSBx)")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--no-init", action="store_true", help="Skip init burst")
    parser.add_argument("--spd", type=int, help="Speed display (0-999)")
    parser.add_argument("--hdg", type=int, help="Heading display (0-999)")
    parser.add_argument("--alt", type=int, help="Altitude display (0-99999)")
    parser.add_argument("--vs", type=int, help="VS/FPA display (signed)")
    parser.add_argument("--baro", type=int, help="Baro display (hPa, 4 digits)")
    parser.add_argument("--backlight", type=int, help="Backlight (0-9999)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    ser.dtr = True
    ser.rts = True

    stop_event = threading.Event()
    thread = threading.Thread(target=reader_thread, args=(ser, stop_event), daemon=True)
    thread.start()

    print(f"Opened {args.port} @ {args.baud}")
    print("Sending handshake: C,")
    ser.write(b"C,")
    time.sleep(0.5)

    if not args.no_init:
        print("Sending init burst...")
        send_tokens(ser, INIT_BURST)
        time.sleep(0.2)

    display_tokens = []
    if args.spd is not None:
        display_tokens.append(f"S{args.spd:03d},")
    if args.hdg is not None:
        display_tokens.append(f"H{args.hdg:03d},")
    if args.alt is not None:
        display_tokens.append(f"A{args.alt:05d},")
    if args.vs is not None:
        display_tokens.append(f"F{args.vs:+05d},")
    if args.baro is not None:
        display_tokens.append(f"#{args.baro:04d},")
    if args.backlight is not None:
        display_tokens.append(f"B{args.backlight:04d},")

    if display_tokens:
        print(f"Sending display tokens: {''.join(display_tokens)}")
        send_tokens(ser, display_tokens)

    print("Ready. Ctrl+C to exit.")
    try:
        while True:
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        ser.close()


if __name__ == "__main__":
    main()
