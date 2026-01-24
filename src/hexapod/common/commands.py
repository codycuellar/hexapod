HIGH_MASK = 0x80
LOW_MASK = 0x7F

# Command high/low is flipped from standard usage since we operate over stdin,
# We cannot use the low bits as raw data, or we risk seinding keyboard interrupts
# to the REPL and crash our program. We intentionally use safe bits for commands
# and then use the upper range for raw data.

# INPUT COMMANDS (Pi → Servo2040)
CMD_SET_SERVO = 0x30  # Set raw servo angles
CMD_SET_LED = 0x33  # Set LED(s)
CMD_SET_RELAY = 0x36  # Enable/disable relay

CMD_GET_STATUS = 0x41  # Request status

# OUTPUT COMMANDS (Servo2040 → Pi)
CMD_RESPONSE = 0x50  # ACK/NACK to SET commands
CMD_STATUS = 0x51  # Status packet
CMD_MESSAGE = 0x52  # Arbitrary log or warning messages

# TWO-WAY
CMD_PING = 0x50  # Pi → Servo2040
CMD_PONG = 0x4F  # Servo2040 → Pi (Pong)

# Response codes (safe ASCII)
ACK = 0x90  # Acknowledge
NACK = 0x91  # Not Acknowledge
