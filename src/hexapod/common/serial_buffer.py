FRAME_START = ord("!")


class SerialPacket:
    __slots__ = ("cmd", "data")

    def __init__(self, cmd: int, data: bytes):
        self.cmd = cmd
        self.data = data


class SerialBuffer:
    STATE_WAITING = 0
    STATE_CMD = 1
    STATE_LENGTH = 2
    STATE_DATA = 3

    @staticmethod
    def decode_hex_byte(value: int):
        if 0x30 <= value <= 0x39:  # '0'–'9'
            return value - 0x30
        if 0x41 <= value <= 0x46:  # 'A'–'F'
            return value - 0x37
        if 0x61 <= value <= 0x66:  # 'a'–'f'
            return value - 0x57
        else:
            raise Exception("Invalid byte detected: {:02X}".format(value))

    @staticmethod
    def encode_hex_byte(value: int):
        if value < 10:
            return 0x30 + value  # '0'–'9'
        else:
            return 0x37 + value  # 'A'–'F'

    @staticmethod
    def hex_bytes_to_byte(high: int, low: int):
        value = SerialBuffer.decode_hex_byte(high) << 4
        value |= SerialBuffer.decode_hex_byte(low)
        return value

    @staticmethod
    def byte_to_hex_bytes(val: int):
        high = SerialBuffer.encode_hex_byte((val >> 4) & 0x0F)
        low = SerialBuffer.encode_hex_byte(val & 0x0F)
        return bytes([high, low])

    def __init__(self):
        self.data = bytearray()
        self.cmd = 0
        self.len = 0
        self.bytes_left = 0
        self.state = self.STATE_WAITING

    def feed(self, byte: int):
        if byte == FRAME_START:
            self._reset()
            self._set_state(self.STATE_CMD)
            return

        if self.state == self.STATE_CMD:
            self.cmd = byte
            self.bytes_left = 2
            self._set_state(self.STATE_LENGTH)
            return

        if self.state == self.STATE_LENGTH:
            if self.bytes_left == 2:
                self.len = byte
                self.bytes_left -= 1
            elif self.bytes_left == 1:
                self.len = self.hex_bytes_to_byte(self.len, byte)
                self.bytes_left = self.len
                self._set_state(self.STATE_DATA)

        elif self.state == self.STATE_DATA:
            self.data.append(byte)
            self.bytes_left -= 1

        if self.bytes_left == 0:
            packet = SerialPacket(self.cmd, bytes(self.data))
            self._reset()
            self._set_state(self.STATE_WAITING)
            return packet

    def _set_state(self, state: int):
        self.state = state

    def _reset(self):
        self.data = bytearray()
        self.cmd = 0
        self.len = 0
        self.bytes_left = 0
        self.state = self.STATE_WAITING
