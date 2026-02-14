import time, math
from dataclasses import dataclass

@dataclass
class LidarPoint:
    x_mm: float
    y_mm: float
    quality: int
    start_flag: int  # 1 = start of revolution (optional)

class RPLidarA1Parser:
    """
    Feed raw bytes -> parse 5-byte nodes -> emit LidarPoint.
    Has data timeout detection for GUI.
    """
    def __init__(self, min_q=10, dist_min=100.0, dist_max=8000.0, data_timeout_s=1.0):
        self.min_q = min_q
        self.dist_min = dist_min
        self.dist_max = dist_max
        self.data_timeout_s = data_timeout_s

        self.buf = bytearray()
        self.last_data_ts = time.time()

    @staticmethod
    def _is_valid_node(n: bytes) -> bool:
        b0 = n[0]
        s = b0 & 0x01
        s_inv = (b0 >> 1) & 0x01
        if (s ^ s_inv) != 1:
            return False
        if (n[1] & 0x01) != 1:  # checkbit must be 1
            return False
        return True

    @staticmethod
    def _parse_node(n: bytes):
        b0, b1, b2, b3, b4 = n
        start_flag = b0 & 0x01
        quality = b0 >> 2

        angle_q6 = ((b2 << 8) | b1) >> 1
        angle_deg = angle_q6 / 64.0

        dist_q2 = (b4 << 8) | b3
        dist_mm = dist_q2 / 4.0
        return start_flag, angle_deg, dist_mm, quality

    def feed(self, chunk: bytes):
        """
        Return list[LidarPoint] parsed from this chunk.
        """
        if chunk:
            self.last_data_ts = time.time()
            self.buf.extend(chunk)

        out = []
        i = 0
        while len(self.buf) - i >= 5:
            n = self.buf[i:i+5]
            if self._is_valid_node(n):
                start, ang, dist, q = self._parse_node(n)
                if q >= self.min_q and self.dist_min <= dist <= self.dist_max:
                    a = math.radians(ang)
                    x = dist * math.cos(a)
                    y = dist * math.sin(a)
                    out.append(LidarPoint(x, y, q, start))
                i += 5
            else:
                i += 1  # resync by 1 byte
        if i > 0:
            del self.buf[:i]
        return out

    def is_data_stale(self) -> bool:
        return (time.time() - self.last_data_ts) > self.data_timeout_s
