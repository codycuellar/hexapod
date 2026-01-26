"""
ASCII protocol serial buffer with state machine parsing.

Protocol: !<CMD><LEN><DATA>

Where:
  ! = frame start (0x21)
  CMD = single ASCII character command code
  LEN = 2 hex ASCII digits indicating an 8 bit int for number of DATA hex chars to follow
  DATA = hex-encoded payload (even number of hex chars)

Example: !P00 (PING with 0 data bytes)
Example: !S08010502lC (Set servo: 8 hex chars = 4 bytes of data)

The parser collects raw bytes. The caller interprets whether those
bytes are hex-encoded ASCII or not.
"""

FRAME_START = ord("!")


class SerialPacket:
    __slots__ = ("cmd", "data")

    def __init__(self, cmd: int, data: bytes):
        self.cmd = cmd
        self.data = data


class SerialBuffer:
    """State machine parser for ASCII serial protocol."""

    STATE_WAITING = 0
    STATE_CMD = 1
    STATE_LENGTH = 2
    STATE_DATA = 3

    @staticmethod
    def hex_char_to_nibble(char: int) -> int:
        """Convert ASCII hex character ('0'-'F') to nibble value (0-15)."""
        if 0x30 <= char <= 0x39:  # '0'–'9'
            return char - 0x30
        if 0x41 <= char <= 0x46:  # 'A'–'F'
            return char - 0x37
        if 0x61 <= char <= 0x66:  # 'a'–'f'
            return char - 0x57
        else:
            raise Exception("Invalid hex char: {:02X}".format(char))

    @staticmethod
    def nibble_to_hex_char(nibble: int) -> int:
        """Convert nibble value (0-15) to ASCII hex character ('0'-'F')."""
        if nibble < 10:
            return 0x30 + nibble  # '0'–'9'
        else:
            return 0x37 + nibble  # 'A'–'F'

    @staticmethod
    def decode_hex_pair(high_char: int, low_char: int) -> int:
        """Convert two hex ASCII characters to byte value (0-255)."""
        high = SerialBuffer.hex_char_to_nibble(high_char)
        low = SerialBuffer.hex_char_to_nibble(low_char)
        return (high << 4) | low

    @staticmethod
    def encode_byte_as_hex(value: int) -> bytes:
        """Convert byte value (0-255) to two hex ASCII characters."""
        high = SerialBuffer.nibble_to_hex_char((value >> 4) & 0x0F)
        low = SerialBuffer.nibble_to_hex_char(value & 0x0F)
        return bytes([high, low])

    @staticmethod
    def encode_bytes_as_hex(data: "bytes | bytearray") -> bytes:
        """Convert multiple bytes to hex-encoded ASCII string."""
        result = bytearray()
        for byte in data:
            result.extend(SerialBuffer.encode_byte_as_hex(byte))
        return bytes(result)

    @staticmethod
    def decode_hex_string(
        hex_data: bytes, expected_bytes: "int | None" = None
    ) -> bytes:
        """
        Decode hex-encoded ASCII string back to raw bytes.

        Args:
            hex_data: Hex-encoded ASCII bytes (e.g., b"0105A3")
            expected_bytes: Optional expected output length for validation

        Returns:
            Decoded raw bytes
        """
        if len(hex_data) % 2 != 0:
            raise ValueError("Hex string must have even length")

        result = bytearray()
        for i in range(0, len(hex_data), 2):
            byte_val = SerialBuffer.decode_hex_pair(hex_data[i], hex_data[i + 1])
            result.append(byte_val)

        if expected_bytes is not None and len(result) != expected_bytes:
            raise ValueError(
                "Expected {} bytes, got {}".format(expected_bytes), len(result)
            )

        return bytes(result)

    @staticmethod
    def build_frame(cmd: int, data: "bytes | bytearray") -> bytes:
        """
        Build a complete protocol frame: !<CMD><LEN><DATA>

        Args:
            cmd: Command byte (ASCII character)
            data: Raw data bytes (will be hex-encoded automatically)

        Returns:
            Complete frame as bytes
        """
        # Hex-encode the data
        hex_data = SerialBuffer.encode_bytes_as_hex(data)

        # Build frame
        frame = bytearray([FRAME_START, cmd])
        frame.extend(SerialBuffer.encode_byte_as_hex(len(hex_data)))
        frame.extend(hex_data)

        return bytes(frame)

    def __init__(self):
        self.data = bytearray()
        self.cmd = 0
        self.len = 0  # Number of hex character bytes to read (not decoded byte count!)
        self.bytes_left = 0
        self.state = self.STATE_WAITING

    def feed(self, byte: int):
        """
        Feed one byte into the parser.
        Returns SerialPacket when complete, None otherwise.
        """
        if byte == FRAME_START:
            self._reset()
            self._set_state(self.STATE_CMD)
            return None

        if self.state == self.STATE_CMD:
            self.cmd = byte
            self.bytes_left = 2  # Expect 2 hex chars for length
            self._set_state(self.STATE_LENGTH)
            return None

        if self.state == self.STATE_LENGTH:
            if self.bytes_left == 2:
                # First hex char of length
                self.len = byte
                self.bytes_left = 1
                return None
            elif self.bytes_left == 1:
                # Second hex char of length - decode it
                self.len = self.decode_hex_pair(self.len, byte)
                # Now self.len is the number of hex character bytes to read
                self.bytes_left = self.len
                self._set_state(self.STATE_DATA)
                return None

        elif self.state == self.STATE_DATA:
            # Read hex character bytes (the actual ASCII hex digits)
            self.data.append(byte)
            self.bytes_left -= 1

        # Check if packet is complete
        if self.state == self.STATE_DATA and self.bytes_left == 0:
            packet = SerialPacket(self.cmd, bytes(self.data))
            self._reset()
            self._set_state(self.STATE_WAITING)
            return packet

        return None

    def _set_state(self, state: int):
        self.state = state

    def _reset(self):
        self.data = bytearray()
        self.cmd = 0
        self.len = 0
        self.bytes_left = 0
        self.state = self.STATE_WAITING
