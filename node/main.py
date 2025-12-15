import time
from reyax_pi import RYLR998
from random import randint

# TODO: Detect OS and set SERIAL_PORT accordingly
SERIAL_PORT = "COM3"
BAUDRATE = 115200

# Must match gateway
# BAND = 868000000
NETWORK_ID = 12
ADDRESS = randint(2, 65535)     # unique node address

# TODO: Scan for gateways and store list of gateways + sort by best RSSI
GATEWAY_ADDR = 1                # gateway address (fixed at 1)

RF_PARAMETERS = (9, 7, 1, 12)
OUTPUT_POWER = 22


def main():
    print("Starting LoRa node...")

    lora = RYLR998(SERIAL_PORT, BAUDRATE)

    if not lora.pulse:
        print("ERROR: RYLR998 not responding")
        return

    print("RYLR998 detected")

    # Configure module
    # lora.band = BAND
    time.sleep(0.1)

    lora.networkid = NETWORK_ID
    time.sleep(0.1)

    lora.address = ADDRESS
    time.sleep(0.1)

    lora.rf_parameters = RF_PARAMETERS
    time.sleep(0.1)

    lora.output_power = OUTPUT_POWER

    # Info
    print("UID:", lora.UID)
    print("Version:", lora.version)
    print("Address:", lora.address)
    print("RF:", lora.rf_parameters)
    print("Power:", lora.output_power)

    print("Node ready")
    print("Type messages to send, Ctrl+C to exit\n")

    # Main loop
    try:
        while True:
            # Check for incoming messages
            msg = lora.receive()

            if msg and msg.data:
                t = msg.data.decode("ascii", errors="ignore")

                if t.startswith("R|"):
                    _, msg_id, payload = t.split("|", 2)
                    # print(f"[RESP] id={msg_id} {payload}")

                    # ACK the response
                    lora.send(GATEWAY_ADDR, f"AR|{msg_id}".encode("ascii"))

            # Non-blocking send
            if input_available():
                data = input("> ")
                if data.strip():
                    msg_id, resp = lora.send_request(GATEWAY_ADDR, data.encode("utf-8"))
                    print(f"[TX] id={msg_id}")
                    print(f"[RESP] id={msg_id} {resp.decode('utf-8', errors='ignore')}")

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nNode stopped")


def input_available():
    """Windows-safe non-blocking stdin check"""
    import msvcrt
    return msvcrt.kbhit()


if __name__ == "__main__":
    main()
