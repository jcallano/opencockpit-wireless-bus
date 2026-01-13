"""
SLIP (Serial Line Internet Protocol) encoder/decoder for Python.

Provides simple, reliable framing for binary data over serial.
Reference: RFC 1055
"""

import serial
from typing import Optional, Generator

# SLIP special characters
SLIP_END = 0xC0
SLIP_ESC = 0xDB
SLIP_ESC_END = 0xDC
SLIP_ESC_ESC = 0xDD


def slip_encode(data: bytes) -> bytes:
    """
    Encode data into a SLIP frame.

    Args:
        data: Raw bytes to encode

    Returns:
        SLIP-encoded bytes with frame delimiters
    """
    encoded = bytearray([SLIP_END])  # Start frame

    for byte in data:
        if byte == SLIP_END:
            encoded.extend([SLIP_ESC, SLIP_ESC_END])
        elif byte == SLIP_ESC:
            encoded.extend([SLIP_ESC, SLIP_ESC_ESC])
        else:
            encoded.append(byte)

    encoded.append(SLIP_END)  # End frame
    return bytes(encoded)


class SlipDecoder:
    """
    Stateful SLIP frame decoder.

    Feed bytes one at a time and check for complete frames.
    """

    def __init__(self, max_size: int = 300):
        self.buffer = bytearray()
        self.escaped = False
        self.max_size = max_size

    def reset(self):
        """Reset decoder state."""
        self.buffer.clear()
        self.escaped = False

    def decode_byte(self, byte: int) -> Optional[bytes]:
        """
        Process a single byte.

        Args:
            byte: Input byte (0-255)

        Returns:
            Complete frame bytes if frame is complete, None otherwise
        """
        if self.escaped:
            self.escaped = False
            if byte == SLIP_ESC_END:
                self.buffer.append(SLIP_END)
            elif byte == SLIP_ESC_ESC:
                self.buffer.append(SLIP_ESC)
            else:
                # Protocol error - store byte anyway
                self.buffer.append(byte)
        else:
            if byte == SLIP_END:
                if len(self.buffer) > 0:
                    frame = bytes(self.buffer)
                    self.reset()
                    return frame
                # Empty frame - ignore
            elif byte == SLIP_ESC:
                self.escaped = True
            else:
                if len(self.buffer) < self.max_size:
                    self.buffer.append(byte)

        return None


class SlipSerial:
    """
    Serial port wrapper with SLIP framing.
    """

    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        """
        Initialize SLIP serial connection.

        Args:
            port: Serial port name (e.g., 'COM3' or '/dev/ttyUSB0')
            baudrate: Baud rate (default 115200)
            timeout: Read timeout in seconds
        """
        self.serial = serial.Serial(port, baudrate, timeout=timeout)
        self.decoder = SlipDecoder()

    def send(self, data: bytes) -> int:
        """
        Send data as a SLIP frame.

        Args:
            data: Raw bytes to send

        Returns:
            Number of bytes written (including SLIP overhead)
        """
        encoded = slip_encode(data)
        return self.serial.write(encoded)

    def receive(self) -> Optional[bytes]:
        """
        Try to receive a complete SLIP frame.

        Returns:
            Complete frame bytes if available, None otherwise
        """
        while self.serial.in_waiting > 0:
            byte = self.serial.read(1)
            if byte:
                frame = self.decoder.decode_byte(byte[0])
                if frame is not None:
                    return frame
        return None

    def receive_blocking(self, timeout: float = 1.0) -> Optional[bytes]:
        """
        Block until a complete frame is received or timeout.

        Args:
            timeout: Maximum time to wait in seconds

        Returns:
            Complete frame bytes if received, None on timeout
        """
        import time
        start = time.time()

        while time.time() - start < timeout:
            frame = self.receive()
            if frame is not None:
                return frame
            time.sleep(0.001)  # Small sleep to avoid busy-waiting

        return None

    def frames(self) -> Generator[bytes, None, None]:
        """
        Generator that yields complete frames as they arrive.

        Yields:
            Complete frame bytes
        """
        while True:
            frame = self.receive()
            if frame is not None:
                yield frame

    def close(self):
        """Close the serial connection."""
        self.serial.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


# Example usage
if __name__ == "__main__":
    # Test encoding/decoding
    original = bytes([0xAA, 0x20, 0x01, 0x02, 0xC0, 0x10, 0xDB, 0x55])
    encoded = slip_encode(original)
    print(f"Original: {original.hex()}")
    print(f"Encoded:  {encoded.hex()}")

    # Decode
    decoder = SlipDecoder()
    for byte in encoded:
        frame = decoder.decode_byte(byte)
        if frame:
            print(f"Decoded:  {frame.hex()}")
            assert frame == original, "Round-trip failed!"

    print("SLIP encode/decode test passed!")
