import time
import json
import hashlib
from dataclasses import dataclass
from reyax_pi import RYLR998, ReceivedMessage
from commands_handler import CommandRegistry
import config


@dataclass
class ScheduledTx:
    send_at: float
    addr: int
    payload: bytes


@dataclass
class PendingResp:
    addr: int
    msg_id: str
    frame: bytes
    next_send_at: float
    attempts: int = 0
    max_attempts: int = 12
    retry_interval_s: float = 0.5


class LoRaGateway:
    def __init__(self):
        self.lora = RYLR998(config.SERIAL_PORT, config.BAUDRATE)
        self.commands = CommandRegistry()
        self.running = False

        # Non-blocking TX scheduling
        self._txq: list[ScheduledTx] = []

        # Non-blocking reliable responses
        self._pending_resps: dict[str, PendingResp] = {}

        self._register_builtin_commands()

    # Init
    def setup(self):
        if not self.lora.pulse:
            raise RuntimeError("RYLR998 not responding")

        time.sleep(0.1)
        self.lora.networkid = config.NETWORK_ID
        time.sleep(0.1)
        self.lora.address = config.ADDRESS
        time.sleep(0.1)
        self.lora.rf_parameters = config.RF_PARAMETERS
        time.sleep(0.1)
        self.lora.output_power = config.OUTPUT_POWER

    # Main Loop

    def start(self):
        self.running = True
        print("LoRa gateway started")

        while self.running:
            # RX
            msg = self.lora.receive()
            if msg:
                try:
                    self.handle_rx(msg)
                except Exception as e:
                    print(f"[ERR] handle_rx: {e!r}")

            # TX scheduled sends (DISC replies, ACKs, etc.)
            self._tick_tx_queue()

            # Retry reliable responses without blocking
            self._tick_pending_responses()

            time.sleep(config.RX_POLL_INTERVAL)

    def stop(self):
        self.running = False

    # Scheduling helpers

    def _schedule_send(self, addr: int, payload: bytes, delay_ms: int = 0):
        send_at = time.monotonic() + (delay_ms / 1000.0)
        self._txq.append(ScheduledTx(send_at=send_at, addr=addr, payload=payload))

    def _tick_tx_queue(self):
        now = time.monotonic()
        if not self._txq:
            return

        remaining: list[ScheduledTx] = []
        for item in self._txq:
            if item.send_at <= now:
                self.lora.send_now(item.addr, item.payload)
            else:
                remaining.append(item)

        self._txq = remaining

    def _queue_reliable_response(self, addr: int, msg_id: str, payload_text: str):
        frame = f"R|{msg_id}|{payload_text}".encode("ascii", errors="ignore")

        self._pending_resps[msg_id] = PendingResp(
            addr=addr,
            msg_id=msg_id,
            frame=frame,
            next_send_at=time.monotonic(),  # send ASAP
            attempts=0,
            max_attempts=12,
            retry_interval_s=0.5,
        )

    def _tick_pending_responses(self):
        now = time.monotonic()
        if not self._pending_resps:
            return

        done = []
        for msg_id, pr in self._pending_resps.items():
            if pr.attempts >= pr.max_attempts:
                print(f"[RESP-FAIL] id={msg_id}")
                done.append(msg_id)
                continue

            if now >= pr.next_send_at:
                self.lora.send_now(pr.addr, pr.frame)

                # quick RX poll burst (listen for node's ACK)
                t_end = time.monotonic() + 0.25
                while time.monotonic() < t_end:
                    m = self.lora.receive()
                    if m and m.data:
                        raw = m.data.decode("ascii", errors="ignore")
                        if raw == f"AR|{pr.msg_id}":
                            print(f"[RESP-ACK] from={m.address} id={pr.msg_id}")
                            done.append(pr.msg_id)
                            break
                    time.sleep(0.01)
                    
                pr.attempts += 1
                pr.next_send_at = now + pr.retry_interval_s
                print(f"[RESP-TX] id={msg_id} attempt={pr.attempts}")

        for msg_id in done:
            self._pending_resps.pop(msg_id, None)

    # Packet handling

    def _disc_slot_delay_ms(self, nonce: str, slots: int, slot_ms: int) -> int:
        key = f"{self.lora.networkid}:{self.lora.address}:{nonce}".encode("utf-8")
        h = hashlib.blake2s(key, digest_size=2).digest()
        slot = int.from_bytes(h, "big") % slots

        # small deterministic jitter within slot
        j = (int.from_bytes(h, "big") % 9) - 4  # [-4..+4] ms
        delay = slot * slot_ms + j

        # clamp to >= 0
        return max(0, delay)

    def handle_rx(self, msg: ReceivedMessage):
        if not msg.data or msg.address is None:
            return

        raw = msg.data.decode("ascii", errors="ignore")

        # 1) Response ACK from node: AR|<msg_id>
        if raw.startswith("AR|"):
            acked_id = raw[3:].strip()
            if acked_id in self._pending_resps:
                print(f"[RESP-ACK] from={msg.address} id={acked_id}")
                self._pending_resps.pop(acked_id, None)
            return

        # 2) Discovery: DISC|<nonce>|<slots>|<slots_ms>
        if raw.startswith("DISC|"):
            parts = raw.split("|", 3)
            if len(parts) != 4:
                return

            _, nonce, slots_s, slot_ms_s = parts

            # safety caps so a malicious DISC wont stall gateway
            slots = max(8, min(256, int(slots_s)))
            slot_ms = max(20, min(100, int(slot_ms_s)))

            delay_ms = self._disc_slot_delay_ms(nonce, slots, slot_ms)

            response = f"GWD|{nonce}".encode("ascii")
            self._schedule_send(msg.address, response, delay_ms=delay_ms)

            print(f"[DISC] from={msg.address} nonce={nonce} delay={delay_ms}ms")
            return

        # 3) Reliable request: D|<msg_id>|<payload>
        if raw.startswith("D|"):
            parts = raw.split("|", 2)
            if len(parts) != 3:
                return

            _, msg_id, payload_str = parts

            # send ACK quickly (scheduled 0ms so we don't block)
            self._schedule_send(msg.address, f"A|{msg_id}".encode("ascii"), delay_ms=0)
            print(f"[ACK] to={msg.address} id={msg_id}")

            # execute command
            try:
                res = self.commands.execute(self, payload_str)
            except Exception as e:
                res = f"ERR {e}"

            if res is None:
                return

            if isinstance(res, bytes):
                res_text = res.decode("latin-1", errors="ignore")
            else:
                res_text = str(res)

            # queue response reliably (non-blocking)
            self._queue_reliable_response(msg.address, msg_id, res_text)
            print(f"[RESP-QUEUED] to={msg.address} id={msg_id} len={len(res_text)}")
            return

        # fallback
        print(f"[RX] from={msg.address} data={msg.data!r}")

    # Commands

    def _register_builtin_commands(self):
        self.commands.register("info", self._cmd_info, help="show module info")

    def _cmd_info(self, _):
        return json.dumps({
            "uid": self.lora.UID,
            "version": self.lora.version,
            "network": self.lora.networkid,
            "address": self.lora.address,
            "rf": self.lora.rf_parameters,
            "power": self.lora.output_power
        }, separators=(",", ":")).encode("utf-8")