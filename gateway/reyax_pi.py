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
        if r != b"+OK\r\n":
            raise Exception(r)

    # Address

    @property
    def address(self) -> int:
        r = self._command_response(b"AT+ADDRESS?\r\n")
        return int(r[9:].decode())

    @address.setter
    def address(self, value: int) -> None:
        r = self._command_response(f"AT+ADDRESS={value}\r\n".encode())
        if r != b"+OK\r\n":
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
        if r != b"+OK\r\n":
            raise Exception(r)

    @property
    def output_power(self) -> int:
        r = self._command_response(b"AT+CRFOP?\r\n")
        return int(r[7:].decode())

    @output_power.setter
    def output_power(self, value: int) -> None:
        r = self._command_response(f"AT+CRFOP={value}\r\n".encode())
        if r != b"+OK\r\n":
            raise Exception(r)

    # TX/RX

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
        address: int,
        payload: bytes,
        *,
        retries: int = 5,
        timeout_s: float = 2.0,
        backoff_s: float = 0.2,
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
                            return msg_id, resp.encode("latin-1", errors="ignore")

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