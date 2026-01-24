HIGH_MASK = 0x80
LOW_MASK = 0x7F
ACK = 0x06
NACK = 0x21

# INPUT COMMANDS (Pi → Servo2040)
CMD_SET_SERVO = 0x80  # Set raw servo angles
CMD_SET_LED = 0x83  # Set LED(s)
CMD_SET_RELAY = 0x86  # Enable/disable relay

CMD_GET_STATUS = 0xA0  # Request status (Servo responds with CMD_STATUS)

# OUTPUT COMMANDS (Servo2040 → Pi)
CMD_RESPONSE = 0xC0  # ACK/NACK to SET commands
CMD_STATUS = 0xC3  # Status packet (voltage, current, flags)
CMD_MESSAGE = 0xC6  # Arbitrary log or warning messages

# TWO-WAY
CMD_PING = 0xF0  # Pi → Servo2040
CMD_PONG = 0xF1  # Servo2040 → Pi
