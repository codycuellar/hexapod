# Command codes (single ASCII characters)
CMD_SET_SERVO = ord("S")  # set [S]ervo positions
# CMD_SET_LED = ord("L")  # set [L]ed state
# CMD_SET_RELAY = ord("R")  # set [R]elay state

CMD_GET_STATUS = ord("Q")  # [Q]uery status

CMD_PING = ord("P")  # [P]ing
CMD_PONG = ord("O")  # p[O]ng response

CMD_RESPONSE = ord("A")  # [A]ck/nack response
CMD_MESSAGE = ord("M")  # [M]essage/log
# CMD_STATUS = ord("T")  # s[T]atus packet

# Response codes
ACK = 0x06
NACK = 0x15
