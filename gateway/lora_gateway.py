import base64
from math import ceil
import shlex
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

        self._pending_cursor = 0
        self._last_pending_addr: int | None = None

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
                    print("handle_rx")
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

    def _tick_tx_queue(self) -> None:
        if not self._txq:
            return

        # GWDs first, then by send_at
        self._txq.sort(key=lambda x: (not x.payload.startswith(b"GWD"), x.send_at))

        iterations = sum(1 for item in self._txq if item.payload.startswith(b"GWD"))
        iterations = iterations + 1 if iterations > 0 else 1

        for _ in range(iterations):
            if not self._txq:
                break

            item = self._txq[0]
            now = time.monotonic()

            # Wait until this item is due (for GWD only)
            if item.send_at > now:
                if item.payload.startswith(b"GWD"):
                    sleep_s = item.send_at - now
                    # safety clamp (optional)
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                else:
                    continue

            try:
                self.lora.send_now(item.addr, item.payload)
                self._txq.pop(0)
            except Exception as e:
                print("[TTXQ] Failed to send item from queue:", e)
                item.send_at = time.monotonic() + 0.25
                break

    def _queue_reliable_response(self, addr: int, msg_id: str, payload_text: str):
        frame = f"R|{msg_id}|{payload_text}".encode("ascii", errors="ignore")

        if len(frame) > 240:
            # Frame is too large, we need to chunk it
            # Calculate number of chunks needed
            # The placeholders here give us enough leeway
            header_size = len(f"RC|{msg_id}|<chk_idx>|<total>|<hash>|")
            payload_size = len(payload_text)
            
            # We are assuming the header is going to be smaller than the max size
            max_p_size_per_chunk = 239 - header_size

            chunks = [
                payload_text[i:i + max_p_size_per_chunk]
                for i in range(0, payload_size, max_p_size_per_chunk)
            ]

            chunks_size = len(chunks)

            i = 0

            for chunk in chunks:
                chunk_id = hashlib.sha1((msg_id + str(i) + str(chunks_size)).encode()).hexdigest()[:8]
                hash = hashlib.sha1(chunk.encode()).hexdigest()[:8]

                chunk_frame = f"R|{chunk_id}|{i}|{chunks_size}|{hash}|{chunk}".encode("ascii", errors="ignore")

                self._pending_resps[chunk_id] = PendingResp(
                    addr=addr,
                    msg_id=chunk_id,
                    frame=chunk_frame,
                    next_send_at=time.monotonic(),
                    attempts=0,
                    max_attempts=12,
                    retry_interval_s=0.5,
                )

                i += 1
            
            return

        self._pending_resps[msg_id] = PendingResp(
            addr=addr,
            msg_id=msg_id,
            frame=frame,
            next_send_at=time.monotonic(),  # send ASAP
            attempts=0,
            max_attempts=12,
            retry_interval_s=0.5,
        )

    def _tick_pending_responses(self) -> None:
        if not self._pending_resps:
            return

        now = time.monotonic()

        MAX_SENDS_PER_TICK = 1

        keys = list(self._pending_resps.keys())
        n = len(keys)
        if n == 0:
            return

        done: list[str] = []
        sent = 0

        start = self._pending_cursor % n

        def _try_pick(avoid_addr: int | None) -> tuple[int, str, PendingResp] | None:
            idx = start

            for _ in range(n):
                msg_id = keys[idx]
                pr = self._pending_resps.get(msg_id)
                if pr is None:
                    idx = (idx + 1) % n
                    continue

                if pr.attempts >= pr.max_attempts:
                    print(f"[RESP-FAIL] id={msg_id}")
                    done.append(msg_id)
                    idx = (idx + 1) % n
                    continue

                if now < pr.next_send_at:
                    idx = (idx + 1) % n
                    continue

                if avoid_addr is not None and pr.addr == avoid_addr:
                    idx = (idx + 1) % n
                    continue

                return idx, msg_id, pr

                # idx advance handled above
            return None

        # Pass 1: avoid repeating the last address
        picked = _try_pick(self._last_pending_addr)
        
        # Pass 2: if we couldn't, allow same address
        if picked is None:
            picked = _try_pick(None)

        if picked is not None and sent < MAX_SENDS_PER_TICK:
            idx, msg_id, pr = picked
            try:
                self.lora.send_now(pr.addr, pr.frame)
                pr.attempts += 1
                pr.next_send_at = time.monotonic() + pr.retry_interval_s
                print(f"[RESP-TX] id={msg_id} attempt={pr.attempts}")
                sent += 1
                self._last_pending_addr = pr.addr
            except Exception as e:
                print("[TPR] Send now error:", e)
                pr.next_send_at = time.monotonic() + 0.25

            # move cursor to the next position after the one we considered
            self._pending_cursor = (idx + 1) % n
        else:
            # nothing due; just advance cursor a bit to avoid bias
            self._pending_cursor = (start + 1) % n

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
    
    def _handle_disc(self, msg: ReceivedMessage):
        if not msg.data or msg.address is None:
            return
        
        raw = msg.data.decode("ascii", errors="ignore")
        
        # Discovery: DISC|<nonce>|<slots>|<slots_ms>
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

    def _handle_exec(self, msg: ReceivedMessage):
        if not msg.data or msg.address is None:
            return
        
        raw = msg.data.decode("ascii", errors="ignore")
        
        # Reliable request: D|<msg_id>|<payload>
        if raw.startswith("D|"):
            parts = raw.split("|", 2)
            if len(parts) != 3:
                return

            _, msg_id, payload_str = parts

            # send ACK quickly (scheduled 0ms so we don't block)
            self._schedule_send(msg.address, f"A|{msg_id}".encode("ascii"), delay_ms=0)
            print(f"[ACK] to={msg.address} id={msg_id}")

            # Extract arguments
            argv = shlex.split(payload_str)

            cmd = argv[0]
            args = argv[1:]

            # execute command
            try:
                res = self.commands.execute(self, cmd, args)
            except Exception as e:
                res = f"ERR {e}"

            if res is None:
                return

            if isinstance(res, bytes):
                res_text = res.decode("latin-1", errors="ignore")
            else:
                res_text = str(res)

            # Encode text as base64
            b64 = base64.b64encode(res_text.encode("utf-8")).decode()

            # queue response reliably (non-blocking)
            self._queue_reliable_response(msg.address, msg_id, b64)
            print(f"[RESP-QUEUED] to={msg.address} id={msg_id} len={len(b64)}")
            return

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
        self._handle_disc(msg)

        # 3) Reliable request: D|<msg_id>|<payload>
        self._handle_exec(msg)

        # fallback
        print(f"[RX] from={msg.address} data={msg.data!r}")

    # Commands

    def _register_builtin_commands(self):
        self.commands.register("info", self._cmd_info, help="show module info")

    def _cmd_info(self, _, *args):
        return json.dumps({
            "uid": self.lora.UID,
            "version": self.lora.version,
            "network": self.lora.networkid,
            "address": self.lora.address,
            "rf": self.lora.rf_parameters,
            "power": self.lora.output_power
        }, separators=(",", ":")).encode("utf-8")