"""
ASCII protocol serial buffer with state machine parsing.

Protocol: !<CMD><LEN><DATA>

Where:
  ! = frame start (0x21)
  CMD = single ASCII character command code
  LEN = 4 hex ASCII digits indicating a u16 count of DATA bytes over the wire
  DATA = Could be UTF-8 or hex encoded ASCII, depending on CMD rules of the serial buffer.

Example: !P00 (PING with 0 data bytes)
Example: !S080105502C (Set servo: 8 hex chars = 4 bytes of data)
Example: !M1BHello this is a log message (1B is 27 - len of text bytes)
"""

try:
    from commands import *
except:
    from .commands import *

FRAME_START = ord("!")


class SerialPacket:
    __slots__ = ("cmd", "data")

    def __init__(self, cmd: int, data: bytearray):
        self.cmd = cmd
        self.data = data


class SerialBuffer:
    """State machine parser for ASCII serial protocol."""

    STATE_WAITING = 0
    STATE_CMD = 1
    STATE_LENGTH = 2
    STATE_DATA = 3

    # 0 for raw bytes (filtered for unsafe bytes)
    # 1 for hex-ascii
    _COMMAND_ENCODING = {CMD_MESSAGE: 0, CMD_SET_SERVO: 1}

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
    def u16_to_bytes(data: int):
        return bytes([(data >> 8) & 0xFF, data & 0xFF])

    @staticmethod
    def build_frame(cmd: int, data: "bytes | bytearray") -> bytes:
        frame = bytearray([FRAME_START, cmd])

        if SerialBuffer._COMMAND_ENCODING.get(cmd) == 1:
            # Hex-encode the data
            data = SerialBuffer.encode_bytes_as_hex(data)

        len_bytes = SerialBuffer.u16_to_bytes(len(data))
        frame.extend(SerialBuffer.encode_bytes_as_hex(len_bytes))
        frame.extend(data)
        return bytes(frame)

    def __init__(self):
        self.data = bytearray()
        self.cmd = 0
        self.packet_len = 0
        self.data_len = 0
        self.packet_bytes_left = 0
        self.prev_hex_char = 0
        self.state = self.STATE_WAITING

    def feed(self, byte: int):
        """
        Feed one byte into the parser. If this byte completes the packet, we return
        the SerialPacket, which is the ASCII hex encoded bytestream.
        """
        if self.state == self.STATE_WAITING:
            if byte == FRAME_START:
                self._reset()
                self._set_state(self.STATE_CMD)

        elif self.state == self.STATE_CMD:
            self.cmd = byte
            self.packet_bytes_left = 4  # Expect 2 hex chars for length
            self._set_state(self.STATE_LENGTH)

        elif self.state == self.STATE_LENGTH:
            nibble = self.hex_char_to_nibble(byte)
            self.packet_len = self.packet_len << 4 | nibble
            self.packet_bytes_left -= 1

            if self.packet_bytes_left == 0:
                # Now self.len is the number of hex character bytes to read
                # If length is 0, packet is complete immediately
                if self.packet_len == 0:
                    packet = SerialPacket(self.cmd, bytearray())
                    self._reset()
                    self._set_state(self.STATE_WAITING)
                    return packet
                else:
                    if self._COMMAND_ENCODING.get(self.cmd) == 1:
                        self.data_len = self.packet_len // 2
                    else:
                        self.data_len = self.packet_len
                    self.packet_bytes_left = self.packet_len
                    self._set_state(self.STATE_DATA)

        elif self.state == self.STATE_DATA:
            if self._COMMAND_ENCODING.get(self.cmd) == 1:
                if self.packet_bytes_left % 2 == 0:
                    self.prev_hex_char = byte
                else:
                    self.data.append(
                        SerialBuffer.decode_hex_pair(self.prev_hex_char, byte)
                    )
            else:
                self.data.append(byte)

            self.packet_bytes_left -= 1

            # Check if packet is complete after reading this byte
            if self.packet_bytes_left == 0:
                if self.data_len != len(self.data):
                    raise ValueError(
                        "Packet bytes len {} does not match header length {}".format(
                            len(self.data), self.data_len
                        )
                    )
                packet = SerialPacket(self.cmd, self.data)
                self._reset()
                self._set_state(self.STATE_WAITING)
                return packet

    def _set_state(self, state: int):
        self.state = state

    def _reset(self):
        self.data = bytearray()
        self.cmd = 0
        self.packet_len = 0
        self.data_len = 0
        self.prev_hex_char = 0
        self.packet_bytes_left = 0
        self.state = self.STATE_WAITING
