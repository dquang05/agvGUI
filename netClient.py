import socket, time, threading, queue
from dataclasses import dataclass

from pyparsing import Optional
from typing import Optional

@dataclass
class AGVInfo:
    name: str
    ip: str
    port_lidar: int
    port_ctrl: int
    Port = int
    port_telem: Port | None = None


class NetClient:
    """
    TCP client:
    - connect timeout
    - recv thread -> rx_queue
    - send()
    - status callback
    """
    def __init__(self, status_cb=None):
        self.status_cb = status_cb or (lambda msg: None)
        self.sock = None
        self.rx_queue = queue.Queue(maxsize=2000)
        self._stop = threading.Event()
        self._rx_thread = None

    # -------- UDP discovery --------
    @classmethod
    def discover_udp(cls, port=50000, timeout=2.5):
        """
        Listen UDP beacon packets and return list[AGVInfo].
        Beacon format: "AGV_01|<ip>|9000|9001"
        """
        found = {}
        t0 = time.time()

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("", port))
            s.settimeout(0.3)

            while time.time() - t0 < timeout:
                try:
                    data, addr = s.recvfrom(256)
                except socket.timeout:
                    continue
                except OSError:
                    break

                text = data.decode("utf-8", errors="ignore").strip()
                parts = text.split("|")
                if len(parts) not in (4, 5):
                    continue
                name, ip, p_lidar, p_ctrl = parts[:4]
                p_telem = parts[4] if len(parts) == 5 else None
                
                
                try:
                    found[ip] = AGVInfo(name=name, ip=ip, port_lidar=int(p_lidar), port_ctrl=int(p_ctrl), port_telem=int(p_telem) if p_telem is not None else None)
                except ValueError:
                    pass

        return list(found.values())

    # -------- TCP connect/disconnect --------
    def connect(self, ip, port, timeout=3.0):
        self.disconnect()
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            s.connect((ip, port))
            s.settimeout(None)
        except OSError as e:
            self.status_cb(f"[NET] connect failed: {e}")
            return False

        self.sock = s
        self._stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, args=(s,), daemon=True)
        self._rx_thread.start()
        self.status_cb(f"[NET] connected to {ip}:{port}")
        return True

    def disconnect(self):
        self._stop.set()
        s = self.sock
        self.sock = None
        try:
            if s:
                s.close()
        except:
            pass

        while not self.rx_queue.empty():
            try:
                self.rx_queue.get_nowait()
            except:
                break

        self.status_cb("[NET] disconnected")

    def send(self, data: bytes):
        s = self.sock
        if not s:
            return False
        try:
            s.sendall(data)
            return True
        except OSError as e:
            self.status_cb(f"[NET] send error: {e}")
            self.disconnect()
            return False

    def _rx_loop(self, s: socket.socket):
        try:
            while not self._stop.is_set():
                data = s.recv(4096)
                if not data:
                    self.status_cb("[NET] server closed")
                    break
                try:
                    self.rx_queue.put_nowait(data)
                except queue.Full:
                    pass
        except OSError as e:
            self.status_cb(f"[NET] recv error: {e}")
        finally:
            self.disconnect()
