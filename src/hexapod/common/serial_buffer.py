# This is common code shared between the host (PC) and the controller (Pico).
# We use a simple class with __slots__ to provide a strongly-typed interface
# that is extremely memory-efficient on the MicroPython board.


class SerialPacket:
    __slots__ = ("cmd", "len", "data")

    def __init__(self, cmd: int, length: int, data: bytearray):
        self.cmd = cmd
        self.len = length
        self.data = data


class SerialBuffer:
    def __init__(self):
        self.buffer = bytearray()
        self.cmd = None
        self.len = 0

    def feed(self, byte: int):
        """
        Feed a single byte into the state machine.
        Protocol: Commands use safe ASCII (0x20-0x7E), data/length use high bits (0x80-0xFF)
        Returns a SerialPacket object if one is completed, else None.
        """
        # Commands are safe ASCII (0x20-0x7E), not control chars (0x00-0x1F) or high bits
        if 0x20 <= byte <= 0x7E and byte & 0x80 == 0:  # Safe ASCII command
            self.reset()
            self.cmd = byte
            return None

        if self.cmd is None:
            return None

        if self.len == 0:
            # Length is encoded in high bits: decode by removing high bit
            if byte & 0x80:
                self.len = byte & 0x7F
                if self.len == 0:
                    return self._finalize()
                return None
            return None  # Invalid - length must have high bit set

        # Data bytes are encoded in high bits: decode by removing high bit
        if byte & 0x80:
            self.buffer.append(byte & 0x7F)
            if len(self.buffer) >= self.len:
                return self._finalize()
        # Ignore bytes without high bit set (they're not part of our protocol)

        return None

    def _finalize(self):
        if not self.cmd:
            return

        msg = SerialPacket(self.cmd, self.len, self.buffer)
        self.reset()
        return msg

    def reset(self):
        self.buffer = bytearray()
        self.cmd = None
        self.len = 0
