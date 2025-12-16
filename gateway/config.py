from random import randint

SERIAL_PORT = "/dev/serial0"
BAUDRATE = 115200

BAND = 868000000
NETWORK_ID = 18

# Pick a random address on startup
# TODO: Check that address is not already in use
ADDRESS = randint(1, 65535)

RF_PARAMETERS = (7, 8, 1, 12)
OUTPUT_POWER = 22

RX_POLL_INTERVAL = 0.01