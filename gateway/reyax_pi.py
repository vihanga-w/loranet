import serial
import time
import threading
import os
import time

class ReceivedMessage:
    def __init__(self) -> None:
        self.address: int | None = None
        self.length: int | None = None
        self.data: bytes | None = None
        self.RSSI: int | None = None
        self.SNR: int | None = None

    def parse(self, full_line: bytes) -> None:
        try:
            i_equal = full_line.find(b"=")
            i_comma1 = full_line.find(b",")
            i_comma2 = full_line.find(b",", i_comma1 + 1)
            i_comma4 = full_line.rfind(b",")
            i_comma3 = full_line.rfind(b",", 0, i_comma4)
            i_linebreak = full_line.find(b"\r\n")

            self.address = int(full_line[i_equal + 1:i_comma1].decode())
            self.length = int(full_line[i_comma1 + 1:i_comma2].decode())
            self.data = full_line[i_comma2 + 1:i_comma3]
            self.RSSI = int(full_line[i_comma3 + 1:i_comma4].decode())
            self.SNR = int(full_line[i_comma4 + 1:i_linebreak].decode())

        except Exception as e:
            raise Exception(f"Unable to parse RCV line {full_line!r}: {e}")

    def __str__(self) -> str:
        return str({
            "address": self.address,
            "length": self.length,
            "data": self.data,
            "RSSI": self.RSSI,
            "SNR": self.SNR
        })

class Node():
    def __init__(self, address: int, rssi: int, snr: int):
        self.address = address
        self.rssi = rssi
        self.snr = snr

    @property
    def quality(self) -> float:
        SNR_MIN = -12.5

        # Link margin dominates
        margin = self.snr - SNR_MIN

        # Normalize RSSI
        rssi_score = max(0.0, min(1.0, (self.rssi + 130) / 60))

        # Weighted quality score
        return (margin * 1.0) + (rssi_score * 5.0)
    
class RYLR998:
    def __init__(self, port="/dev/serial0", baudrate=115200, timeout=0.1):
        self._uart = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout
        )

        self._uart.reset_input_buffer()
        self._rxbuf = b""
        self._lock = threading.RLock()
        self.links = []

    # Basic Info

    @property
    def pulse(self) -> bool:
        r = self._command_response(b"AT\r\n")

        return b"OK" in r

    @property
    def UID(self) -> str:
        return self._command_response(b"AT+UID?\r\n")[5:-2].decode()

    @property
    def version(self) -> str:
        return self._command_response(b"AT+VER?\r\n")[5:-2].decode()

    # Network

    @property
    def networkid(self) -> int:
        r = self._command_response(b"AT+NETWORKID?\r\n")
        if b"+NETWORKID=" not in r:
            raise Exception(f"Bad response: {r}")
        return int(r[11:].decode())

    @networkid.setter
    def networkid(self, value: int) -> None:
        valid = {3,4,5,6,7,8,9,10,11,12,13,14,15,18}
        if value not in valid:
            raise ValueError(f"Invalid network ID {value}")
        r = self._command_response(f"AT+NETWORKID={value}\r\n".encode())
        if not self._is_ok(r):
            raise Exception(r)

    # Address

    @property
    def address(self) -> int:
        r = self._command_response(b"AT+ADDRESS?\r\n")
        return int(r[9:].decode())

    @address.setter
    def address(self, value: int) -> None:
        r = self._command_response(f"AT+ADDRESS={value}\r\n".encode())
        if not self._is_ok(r):
            raise Exception(r)

    # RF

    @property
    def rf_parameters(self):
        r = self._command_response(b"AT+PARAMETER?\r\n")
        params = r[11:].decode().strip().split(",")
        return tuple(map(int, params))

    @rf_parameters.setter
    def rf_parameters(self, value):
        s = ",".join(map(str, value))
        r = self._command_response(f"AT+PARAMETER={s}\r\n".encode())
        if not self._is_ok(r):
            raise Exception(r)

    @property
    def output_power(self) -> int:
        r = self._command_response(b"AT+CRFOP?\r\n")
        return int(r[7:].decode())

    @output_power.setter
    def output_power(self, value: int) -> None:
        r = self._command_response(f"AT+CRFOP={value}\r\n".encode())
        if not self._is_ok(r):
            raise Exception(r)

    # TX/RX

    def pick_ideal_gw(self) -> int:
        if not self.links:
            raise Exception("No known gateways")

        best_link = max(self.links, key=lambda link: link.quality)
        
        return best_link.address

    def send(self, address: int, data: bytes) -> None:
        if len(data) > 240:
            raise ValueError("Payload too large")

        cmd = b"AT+SEND=" + \
            str(address).encode() + b"," + \
            str(len(data)).encode() + b"," + \
            data + b"\r\n"

        r = self._command_response(cmd, 8.0)
        if b"OK" not in r:
            raise Exception(r)

    def send_request(
        self,
        payload: bytes,
        *,
        retries: int = 5,
        timeout_s: float = 2.0,
        backoff_s: float = 0.2,
        address_override: int | None = None,
    ):
        """
        Reliable request/response:
        - sends D|id|payload
        - waits for either A|id or R|id|...
        - if R|id|... arrives, we ACK it with AR|id and return the response payload
        - retries sending D|... if neither arrives
        Returns: (msg_id, response_bytes)
        """
        msg_id = self._new_msg_id()
        address = address_override if address_override is not None else self.pick_ideal_gw()

        body = payload.decode("latin-1", errors="ignore")
        frame = f"D|{msg_id}|{body}".encode("ascii", errors="ignore")

        for attempt in range(retries):
            # send request
            self.send(address, frame)

            # wait for ACK or RESPONSE
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                msg = self.receive()
                if not msg or not msg.data:
                    time.sleep(0.01)
                    continue

                t = msg.data.decode("ascii", errors="ignore")

                # If we got an ACK, keep waiting for response until deadline
                if t == f"A|{msg_id}":
                    continue

                # If we got the response, ACK it and return
                if t.startswith("R|"):
                    parts = t.split("|", 2)
                    if len(parts) == 3:
                        _, rid, resp = parts
                        if rid == msg_id:
                            # ACK response so gateway can stop retrying
                            self.send(address, f"AR|{msg_id}".encode("ascii"))
                            return msg_id, resp.encode("latin-1", errors="ignore"), msg

            # retry with backoff
            time.sleep(backoff_s * (attempt + 1))

        raise TimeoutError(f"send_request failed after {retries} retries; msg_id={msg_id}")

    def receive(self):
        with self._lock:
            self._collect_rx()

            i1 = self._rxbuf.find(b"+RCV=")
            if i1 == -1:
                return None

            i2 = self._rxbuf.find(b"\r\n", i1)
            if i2 == -1:
                return None  # wait for full line

            frame = self._rxbuf[i1:i2+2]
            self._rxbuf = self._rxbuf[:i1] + self._rxbuf[i2+2:]

            msg = ReceivedMessage()
            msg.parse(frame)
            
            return msg
    
    def refresh_link_quality(self):
        for link in self.links:
            _, _, msg = self.send_request(b"PING", address_override=link.address)

            # Update signal quality info
            link.rssi = msg.RSSI if msg.RSSI is not None else link.rssi
            link.snr = msg.SNR if msg.SNR is not None else link.snr

            print(f"Link to {link.address}: RSSI={link.rssi} dBm, SNR={link.snr}, quality={link.quality:.1f}")

    def discover_gw(self):
        while True:
            disc_nonce = os.urandom(4).hex()

            discovered_gateways = []
            slots = 125
            slots_ms = 30
            max_wait = ((slots * slots_ms) / 1000) + 2

            print("Discovering gateways, this will take", max_wait, "seconds...")

            self.send(0, f"DISC|{disc_nonce}|{slots}|{slots_ms}".encode("ascii"))

            start_time = time.time()

            try:
                while True:
                    if time.time() - start_time > max_wait:
                        break

                    msg = self.receive()

                    # Ignore invalid responses
                    if not msg or not msg.data or not msg.address or not msg.RSSI or not msg.SNR:
                        time.sleep(0.01)
                        continue

                    if msg and msg.data:
                        t = msg.data.decode("ascii", errors="ignore")

                        if t.startswith("GWD|"):
                            _, nonce = t.split("|", 1)

                            if nonce == disc_nonce:
                                link = Node(msg.address, msg.RSSI, msg.SNR)

                                discovered_gateways.append(link)

                                print(f"Discovered gateway at address {link.address} with RSSI {link.rssi} dBm, SNR {link.snr}, quality {link.quality:.1f}")
                
                if len(discovered_gateways) == 0:
                    print("No gateways found, rescanning...")
                    continue

                self.links = discovered_gateways

                print("Discovery complete. Found", len(discovered_gateways))

                break
            except KeyboardInterrupt:
                print("Discovery interrupted")
                return

    # Internal methods

    def _collect_rx(self):
        # read everything currently available
        n = self._uart.in_waiting
        if n:
            self._rxbuf += self._uart.read(n)

    def _command_response(self, cmd: bytes, timeout_s=0.5) -> bytes:
        with self._lock:
            # pull any async +RCV into buffer first
            self._collect_rx()

            self._uart.write(cmd)

            start = time.monotonic()
            buf = b""

            while time.monotonic() - start < timeout_s:
                n = self._uart.in_waiting
                if n:
                    buf += self._uart.read(n)

                    # accept either \r\n or \r as terminator
                    term = b"\r\n" if b"\r\n" in buf else (b"\r" if b"\r" in buf else None)
                    if term is None:
                        continue

                    line, buf = buf.split(term, 1)
                    line += term

                    if line.startswith(b"+RCV="):
                        self._rxbuf += line
                        continue

                    return line

                time.sleep(0.001)

            raise TimeoutError(f"No response for {cmd!r}, partial={buf!r}")
    
    def _new_msg_id(self) -> str:
        # 4 bytes -> 8 hex chars, short but good enough for testing
        return os.urandom(4).hex()
    
    def _is_ok(self, r: bytes) -> bool:
        # tolerate '+OK\r', '+OK\r\n', 'OK\r\n', etc.
        return r.strip() in (b"+OK", b"OK")

    def send_reliable(
        self,
        address: int,
        payload: bytes,
        *,
        retries: int = 5,
        ack_timeout_s: float = 1.2,
        backoff_s: float = 0.15,
    ) -> str:
        """
        Sends payload reliably using app-level ACK.
        Returns msg_id if ACKed, raises TimeoutError otherwise.
        Payload is bytes; will be encoded as latin-1 into ASCII-safe transport.
        """
        msg_id = self._new_msg_id()

        # Keep it ASCII-safe: payload bytes -> latin-1 text (1:1 mapping)
        body = payload.decode("latin-1", errors="ignore")
        frame = f"D|{msg_id}|{body}".encode("ascii", errors="ignore")

        last_err = None
        for attempt in range(retries):
            try:
                self.send(address, frame)
            except Exception as e:
                last_err = e

            # wait for ACK
            if self._wait_for_ack(msg_id, ack_timeout_s):
                return msg_id

            time.sleep(backoff_s * (attempt + 1))

        if last_err:
            raise TimeoutError(f"send_reliable failed after {retries} retries; last send error: {last_err}")
        raise TimeoutError(f"send_reliable failed after {retries} retries; no ACK for msg_id={msg_id}")

    def _wait_for_ack(self, msg_id: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            msg = self.receive()
            if not msg or not msg.data:
                time.sleep(0.01)
                continue

            # msg.data is bytes; ACK frame is ASCII
            try:
                text = msg.data.decode("ascii", errors="ignore")
            except Exception:
                continue

            if text.startswith("A|") and text[2:].strip() == msg_id:
                return True

        return False