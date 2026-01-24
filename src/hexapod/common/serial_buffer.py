class SerialBuffer:
    def __init__(self, input_buffer):
        self.input_buffer = input_buffer

        self.buffer = bytearray()
        self.cmd = None
        self.len = 0

    def read_bytes(self):
        """
        Poll for available data and read all available bytes from stream.
        Returns number of bytes read.
        """
        while True:
            try:
                byte = self.input_buffer.read(1)

                if not byte:
                    break

                byte = byte[0]

                if byte & 0x80:  # it's a command
                    if self.cmd:
                        self.reset()
                    self.cmd = byte
                else:  # it's not a command
                    byte = byte & 0x7F
                    if not self.cmd:
                        # skip this byte if we haven't received a command yet
                        continue

                    if not self.len:
                        self.len = byte
                    else:
                        self.buffer.append(byte)
            except:
                break

        if len(self.buffer) == self.len:
            msg = {"cmd": self.cmd, "len": self.len, "data": self.buffer}
            self.reset()
            return msg

    def reset(self):
        self.buffer = bytearray()
        self.cmd = None
        self.len = 0
