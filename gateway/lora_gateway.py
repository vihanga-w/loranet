import time
import json
from reyax_pi import RYLR998, ReceivedMessage
from commands_handler import CommandRegistry
import config

class LoRaGateway:
    def __init__(self):
        self.lora = RYLR998(config.SERIAL_PORT, config.BAUDRATE)
        self.commands = CommandRegistry()
        self.running = False

        self._register_builtin_commands()

    # Init

    def setup(self):
        if not self.lora.pulse:
            raise RuntimeError("RYLR998 not responding")

        # self.lora.band = config.BAND
        time.sleep(0.1)

        self.lora.networkid = config.NETWORK_ID
        time.sleep(0.1)

        self.lora.address = config.ADDRESS
        time.sleep(0.1)

        self.lora.rf_parameters = config.RF_PARAMETERS
        time.sleep(0.1)

        self.lora.output_power = config.OUTPUT_POWER

        # TODO: Check and validate configuration and retry if necessary

    # RX Loop

    def start(self):
        self.running = True
        print("LoRa gateway started")

        while self.running:
            msg = self.lora.receive()
            if msg:
                self.handle_rx(msg)

            time.sleep(config.RX_POLL_INTERVAL)

    def stop(self):
        self.running = False

    # Packet handling

    def handle_rx(self, msg: ReceivedMessage):
        # No data or invalid address
        if not msg.data or msg.address is None:
            return

        raw = msg.data.decode("ascii", errors="ignore")

        # Discovery frame (DISC|<nonce>|<slots>|<slots_ms>)
        if raw.startswith("DISC|"):
            parts = raw.split("|", 3)

            if len(parts) != 4:
                return

            _, nonce, slots, slots_ms = parts

            gw_hash = hash((self.lora.networkid, self.lora.address, nonce))

            slot_number = gw_hash % int(slots)
            response_delay_ms = slot_number * int(slots_ms)

            # Incorporate some jitter to reduce impact of collisions
            jitter_ms = (gw_hash % 10) - 5  # +/-5ms
            
            # Invert jitter such that delay + jitter is always within slot time
            if response_delay_ms + jitter_ms > int(slots) * int(slots_ms):
                jitter_ms = -jitter_ms
            
            response_delay_ms += jitter_ms

            print(f"[DISC] from={msg.address} nonce={nonce} slot={slot_number} delay={response_delay_ms}ms")

            response = f"GWD|{nonce}"
            self.lora.send(msg.address, response.encode("ascii"))

            print(f"[DISC] from={msg.address} nonce={nonce} slots={slots} slots_ms={slots_ms}")
            print(f"[DISC-R] to={msg.address} resp={response}")

            return

        # Reliable DATA frame
        if raw.startswith("D|"):
            parts = raw.split("|", 2)
            if len(parts) != 3:
                return

            _, msg_id, payload_str = parts

            # Send ACK immediately
            self.lora.send(msg.address, f"A|{msg_id}".encode("ascii"))
            print(f"[ACK] to={msg.address} id={msg_id}")

            # Payload becomes the command text
            command_text = payload_str

            try:
                res = self.commands.execute(self, command_text)
            except Exception as e:
                res = f"ERR {e}".encode("utf-8")

            # Send response correlated to msg_id
            if res is None:
                return
            if isinstance(res, bytes):
                res_text = res.decode("latin-1", errors="ignore")
            else:
                res_text = str(res)

            self.send_response_reliable(msg.address, msg_id, res_text)

            print(f"[TX] to={msg.address} id={msg_id} resp_len={len(res_text)}")
            
            return

        print(f"[RX] from={msg.address} data={msg.data}")

    def send_response_reliable(self, addr, msg_id, payload, retries=15, timeout_s=0.25):
        frame = f"R|{msg_id}|{payload}".encode("ascii", errors="ignore")

        for attempt in range(retries):
            self.lora.send(addr, frame)

            deadline = time.monotonic() + timeout_s

            while time.monotonic() < deadline:
                msg = self.lora.receive()

                if not msg or not msg.data:
                    time.sleep(0.01)
                    continue

                t = msg.data.decode("ascii", errors="ignore")

                if t == f"AR|{msg_id}":
                    print(f"[RESP-ACK] from={addr} id={msg_id}")
                    return

            print(f"[RESP-RETRY] id={msg_id} attempt={attempt+1}")

        print(f"[RESP-FAIL] id={msg_id}")

    # Commands

    def _register_builtin_commands(self):
        # self.commands.register(
        #     "send",
        #     self._cmd_send,
        #     help="send <addr> <data>"
        # )

        self.commands.register(
            "info",
            self._cmd_info,
            help="show module info"
        )

        # self.commands.register(
        #     "set_power",
        #     self._cmd_set_power,
        #     help="set_power <0-22>"
        # )

    # def _cmd_send(self, _, addr, data):
    #     self.lora.send(int(addr), data.encode())
    #     return "OK"

    def _cmd_info(self, _):
        return json.dumps({
            "uid": self.lora.UID,
            "version": self.lora.version,
            "network": self.lora.networkid,
            "address": self.lora.address,
            "rf": self.lora.rf_parameters,
            "power": self.lora.output_power
        }).encode('utf-8')

    def _cmd_set_power(self, _, value):
        self.lora.output_power = int(value)
        return "OK"