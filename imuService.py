import threading, time, struct
from collections import deque
from typing import Optional, List, Dict

from netClient import NetClient

MAGIC = 0xA55A
PKT_SIZE = 32
FMT = struct.Struct("<HBBIIhhhhhhhhhh")
MAGIC_BYTES = b"\x5A\xA5"  # 0xA55A little-endian


class ImuStreamParser:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk: bytes) -> List[Dict]:
        if chunk:
            self.buf.extend(chunk)

        out = []
        while True:
            idx = self.buf.find(MAGIC_BYTES)
            if idx < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                break

            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < PKT_SIZE:
                break

            raw = bytes(self.buf[:PKT_SIZE])
            del self.buf[:PKT_SIZE]

            try:
                fields = FMT.unpack(raw)
            except struct.error:
                continue

            magic, ver, typ, seq, t_ms, *vals = fields
            if magic != MAGIC or ver != 1 or typ != 1:
                continue

            out.append({
                "seq": int(seq),
                "t_ms": int(t_ms),
                "roll_deg": vals[0] / 100.0,
                "pitch_deg": vals[1] / 100.0,
                "ax_g": vals[2] / 1000.0,
                "ay_g": vals[3] / 1000.0,
                "az_g": vals[4] / 1000.0,
                "gx_dps": vals[5] / 10.0,
                "gy_dps": vals[6] / 10.0,
                "gz_dps": vals[7] / 10.0,
                "temp_c": vals[8] / 10.0,
            })

        return out


class ImuService:
    """
    IMU background reader:
    - connect via NetClient
    - parse packets
    - store into ring buffer (deque)
    - expose local API: get_latest(), get_since_seq(), status()
    """
    def __init__(self, status_cb=None, buf_max=5000):
        self.status_cb = status_cb or (lambda msg: None)
        self.net = NetClient(status_cb=self.status_cb)
        self.parser = ImuStreamParser()

        self._buf = deque(maxlen=buf_max)
        self._lock = threading.Lock()

        self._stop = threading.Event()
        self._th = None

        self._connected = False
        self._rx_packets = 0
        self._last_seq: Optional[int] = None
        self._last_rx_time = 0.0

    # ---------- lifecycle ----------
    def start(self, ip: str, port: int = 9002, timeout=3.0) -> bool:
        self.stop()

        ok = self.net.connect(ip, port, timeout=timeout)
        self._connected = ok
        self._last_rx_time = time.time()

        if not ok:
            return False

        self._stop.clear()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._th.start()
        return True

    def stop(self):
        self._stop.set()
        try:
            self.net.disconnect()
        except:
            pass
        self._connected = False

    # ---------- internal loop ----------
    def _loop(self):
        while not self._stop.is_set():
            try:
                chunk = self.net.rx_queue.get_nowait()  # allow stop check
            except:
                time.sleep(0.001)
                continue

            if not chunk:
                continue

            pkts = self.parser.feed(chunk)
            if not pkts:
                continue

            now = time.time()
            with self._lock:
                for p in pkts:
                    self._buf.append(p)
                    self._last_seq = p["seq"]
                    self._rx_packets += 1
                self._last_rx_time = now

    # ---------- API ----------
    def get_latest(self, n: int = 200) -> List[Dict]:
        n = max(1, min(2000, int(n)))
        with self._lock:
            return list(self._buf)[-n:]

    def get_since_seq(self, seq: int) -> List[Dict]:
        # returns packets with p["seq"] > seq
        with self._lock:
            if not self._buf:
                return []
            out = [p for p in self._buf if p["seq"] > seq]
            return out

    def status(self) -> Dict:
        with self._lock:
            return {
                "connected": bool(self._connected and self.net.sock),
                "buf_len": len(self._buf),
                "rx_packets": int(self._rx_packets),
                "last_seq": self._last_seq,
                "last_rx_age_s": float(time.time() - self._last_rx_time),
            }
