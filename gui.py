import threading, time, struct
from collections import deque

from matplotlib import text
import numpy as np
import tkinter as tk
from tkinter import ttk, messagebox

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from netClient import NetClient
from lidarParser import RPLidarA1Parser

PORT_LIDAR = 9000
PORT_CTRL  = 9001

CMD_STOP  = 0xFF
CMD_DRIVE = 0xD1  # + int16 v, int16 w (little-endian)

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AGV GUI")
        self.geometry("980x700")

        # networking
        self.net_lidar = NetClient(status_cb=self._status)
        self.net_ctrl  = NetClient(status_cb=self._status)

        # parser
        self.parser = RPLidarA1Parser(min_q=10, dist_min=100, dist_max=8000, data_timeout_s=1.0)

        # plot buffers
        self.max_points = 2000
        self.xs = deque(maxlen=self.max_points)
        self.ys = deque(maxlen=self.max_points)

        self.frame_mode = tk.BooleanVar(value=True)

        self._build_ui()

        # periodic GUI tasks
        self.after(30, self._pump_lidar_bytes)   # parse fast
        self.after(60, self._update_plot)        # draw ~16 Hz
        self.after(300, self._update_status)     # status every 0.3s

    def _build_ui(self):
        self._scan_running = False
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")

        ttk.Button(top, text="Scan AGV", command=self.on_scan).pack(side="left")
        ttk.Button(top, text="Connect", command=self.on_connect).pack(side="left", padx=6)
        ttk.Button(top, text="Disconnect", command=self.on_disconnect).pack(side="left", padx=6)

        ttk.Checkbutton(top, text="Frame-by-frame (reset each revolution)",
                        variable=self.frame_mode).pack(side="left", padx=12)

        ttk.Button(top, text="STOP", command=self.on_stop).pack(side="right")
        ttk.Button(top, text="Drive test (v=200,w=0)", command=lambda: self.on_drive(200,0)).pack(side="right", padx=6)
        ttk.Button(top, text="Turn test (v=0,w=60)", command=lambda: self.on_drive(0,60)).pack(side="right", padx=6)

        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(self, textvariable=self.status_var, padding=(8,0)).pack(anchor="w")

        mid = ttk.Frame(self, padding=8)
        mid.pack(fill="both", expand=True)

        left = ttk.Frame(mid)
        left.pack(side="left", fill="y")

        ttk.Label(left, text="Found devices (:9000)").pack(anchor="w")
        self.listbox = tk.Listbox(left, height=10, width=22)
        self.listbox.pack(fill="y", pady=6)
        self.listbox.bind("<<ListboxSelect>>", self._on_select)

        ttk.Label(left, text="IP").pack(anchor="w", pady=(10,0))
        self.ip_entry = ttk.Entry(left, width=22)
        self.ip_entry.pack(anchor="w")

        right = ttk.Frame(mid)
        right.pack(side="left", fill="both", expand=True, padx=(10,0))

        fig = Figure(figsize=(6.4, 5.4), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlim(-6000, 6000)
        self.ax.set_ylim(-6000, 6000)
        self.ax.set_title("RPLidar raw over TCP (mm)")
        self.ax.set_xlabel("x (mm)")
        self.ax.set_ylabel("y (mm)")
        self.sc = self.ax.scatter([], [], s=3)

        self.canvas = FigureCanvasTkAgg(fig, master=right)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    # ---------- status ----------
    def _status(self, msg: str):
        # called from threads -> marshal to GUI thread
        self.after(0, lambda: self.status_var.set(msg))

    def _update_status(self):
        stale = self.parser.is_data_stale()
        if stale and self.net_lidar.sock:
            # connected but no data
            self.status_var.set("[LIDAR] timeout: no data")
        self.after(300, self._update_status)

    # ---------- scan ----------
    def on_scan(self):
        if self._scan_running:
            self.status_var.set("[SCAN] already running...")
            return

        self._scan_running = True
        self.status_var.set("[SCAN] listening for AGV beacons...")
        self.listbox.delete(0, tk.END)
        def worker():
            try:
                found = NetClient.discover_udp(port=50000, timeout=2.5)
                self.after(0, lambda: self._show_scan(found))
            finally:
                self._scan_running = False

        threading.Thread(target=worker, daemon=True).start()


    def _show_scan(self, found):
        for agv in found:
            self.listbox.insert(tk.END, f"{agv.name} {agv.ip} {agv.port_lidar} {agv.port_ctrl}")
        self.status_var.set(f"[SCAN] found {len(found)} AGV(s).")


    def _on_select(self, _):
        sel = self.listbox.curselection()
        if not sel:
            return
        text = self.listbox.get(sel[0])
        ip = text.split()[1]  # name ip lidar ctrl

        self.ip_entry.delete(0, tk.END)
        self.ip_entry.insert(0, ip)

    # ---------- connect/disconnect ----------
    def on_connect(self):
        ip = self.ip_entry.get().strip()
        if not ip:
            messagebox.showwarning("No IP", "Select an IP or type it.")
            return

        ok1 = self.net_lidar.connect(ip, PORT_LIDAR, timeout=3.0)
        ok2 = self.net_ctrl.connect(ip, PORT_CTRL, timeout=3.0)

        if ok1:
            self.xs.clear(); self.ys.clear()
            self.status_var.set(f"[NET] connected lidar+ctrl to {ip}")
        else:
            self.status_var.set("[NET] lidar connect failed")

        if not ok2:
            self.status_var.set("[NET] ctrl connect failed (lidar may still work)")

    def on_disconnect(self):
        self.net_lidar.disconnect()
        self.net_ctrl.disconnect()
        self.status_var.set("[NET] disconnected")

    # ---------- control send ----------
    def on_stop(self):
        # send 1 byte 0xFF
        self.net_ctrl.send(bytes([CMD_STOP]))

    def on_drive(self, v_mm_s: int, w_deg_s: int):
        # pack: hdr + int16(v) + int16(w) little-endian
        payload = struct.pack("<Bhh", CMD_DRIVE, int(v_mm_s), int(w_deg_s))
        self.net_ctrl.send(payload)

    # ---------- lidar pump + parse ----------
    def _pump_lidar_bytes(self):
        # pull raw bytes from net queue and feed parser
        pulled = 0
        while pulled < 50:  # limit per tick
            try:
                chunk = self.net_lidar.rx_queue.get_nowait()
            except:
                break
            points = self.parser.feed(chunk)
            for p in points:
                if self.frame_mode.get() and p.start_flag == 1:
                    self.xs.clear(); self.ys.clear()
                self.xs.append(p.x_mm)
                self.ys.append(p.y_mm)
            pulled += 1

        self.after(30, self._pump_lidar_bytes)

    # ---------- plot update ----------
    def _update_plot(self):
        if len(self.xs) > 0:
            arr = np.column_stack((np.fromiter(self.xs, float), np.fromiter(self.ys, float)))
            self.sc.set_offsets(arr)
            self.canvas.draw_idle()
        self.after(60, self._update_plot)

if __name__ == "__main__":
    App().mainloop()
