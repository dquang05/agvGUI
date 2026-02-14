import tkinter as tk
from tkinter import ttk, messagebox
from collections import deque
import numpy as np

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from imuService import ImuService


class ImuApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("IMU Realtime")
        self.geometry("1050x750")

        self.imu = ImuService(status_cb=self._status)
        self._last_pkt = None

        self.max_points = 600  # ~6s at 100Hz
        self.t = deque(maxlen=self.max_points)

        self.roll = deque(maxlen=self.max_points)
        self.pitch = deque(maxlen=self.max_points)
        self.yaw = deque(maxlen=self.max_points)
        self._yaw_deg = 0.0


        self.ax = deque(maxlen=self.max_points)
        self.ay = deque(maxlen=self.max_points)
        self.az = deque(maxlen=self.max_points)

        self.gx = deque(maxlen=self.max_points)
        self.gy = deque(maxlen=self.max_points)
        self.gz = deque(maxlen=self.max_points)

        self._last_seq = -1
        self._t0_ms = None

        self._build_ui()

        self.after(30, self._pump_imu)     # parse fast
        self.after(60, self._update_plot)  # draw ~16Hz
        self.after(300, self._update_statline)

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="ESP IP:").pack(side="left")
        self.ip_entry = ttk.Entry(top, width=18)
        self.ip_entry.insert(0, "192.168.1.10")
        self.ip_entry.pack(side="left", padx=6)

        ttk.Label(top, text="Port:").pack(side="left")
        self.port_entry = ttk.Entry(top, width=8)
        self.port_entry.insert(0, "9002")
        self.port_entry.pack(side="left", padx=6)

        ttk.Button(top, text="Connect", command=self.on_connect).pack(side="left", padx=6)
        ttk.Button(top, text="Disconnect", command=self.on_disconnect).pack(side="left", padx=6)
        ttk.Button(top, text="Clear", command=self.on_clear).pack(side="left", padx=6)

        self.status_var = tk.StringVar(value="Ready.")
        ttk.Label(self, textvariable=self.status_var, padding=(8,0)).pack(anchor="w")

        mid = ttk.Frame(self, padding=8)
        mid.pack(fill="both", expand=True)

        fig = Figure(figsize=(8.4, 6.2), dpi=100)
        self.ax1 = fig.add_subplot(311)
        self.ax2 = fig.add_subplot(312, sharex=self.ax1)
        self.ax3 = fig.add_subplot(313, sharex=self.ax1)

        self.ax1.set_ylabel("deg")
        self.ax1.set_title("Roll / Pitch")
        self.l_roll, = self.ax1.plot([], [], label="roll")
        self.l_pitch, = self.ax1.plot([], [], label="pitch")
        self.l_yaw, = self.ax1.plot([], [], label="yaw")
        self.ax1.legend(loc="upper right")

        self.ax2.set_ylabel("g")
        self.ax2.set_title("Accel")
        self.l_ax, = self.ax2.plot([], [], label="ax")
        self.l_ay, = self.ax2.plot([], [], label="ay")
        self.l_az, = self.ax2.plot([], [], label="az")
        self.ax2.legend(loc="upper right")

        self.ax3.set_ylabel("dps")
        self.ax3.set_xlabel("t (s)")
        self.ax3.set_title("Gyro")
        self.l_gx, = self.ax3.plot([], [], label="gx")
        self.l_gy, = self.ax3.plot([], [], label="gy")
        self.l_gz, = self.ax3.plot([], [], label="gz")
        self.ax3.legend(loc="upper right")

        self.canvas = FigureCanvasTkAgg(fig, master=mid)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def _status(self, msg: str):
        self.after(0, lambda: self.status_var.set(msg))

    def on_connect(self):
        ip = self.ip_entry.get().strip()
        if not ip:
            messagebox.showwarning("No IP", "Type ESP IP.")
            return
        try:
            port = int(self.port_entry.get().strip())
        except:
            messagebox.showwarning("Bad port", "Port must be integer.")
            return

        ok = self.imu.start(ip, port=port, timeout=3.0)
        if ok:
            self.status_var.set(f"[IMU] connected to {ip}:{port}")
        else:
            self.status_var.set("[IMU] connect failed")

    def on_disconnect(self):
        self.imu.stop()
        self.status_var.set("[IMU] disconnected")

    def on_clear(self):
        self._last_seq = -1
        self._t0_ms = None
        self.yaw.clear()
        self._yaw_deg = 0.0
        self.t.clear()
        self.roll.clear(); self.pitch.clear()
        self.ax.clear(); self.ay.clear(); self.az.clear()
        self.gx.clear(); self.gy.clear(); self.gz.clear()
        self.status_var.set("[IMU] cleared buffers")

    def _pump_imu(self):
        # Pull only new packets since last seq to reduce copy cost
        new = self.imu.get_since_seq(self._last_seq)

        for p in new:
            self._last_pkt = p
            self._last_seq = p["seq"]
            if self._t0_ms is None:
                self._t0_ms = p["t_ms"]

            tt = (p["t_ms"] - self._t0_ms) / 1000.0
            if len(self.t) > 0:
                dt = tt - self.t[-1]
                if dt < 0: dt = 0
                if dt > 0.1: dt = 0.1
            else:
                dt = 0.0
            self.t.append(tt)

            gz = p["gz_dps"]
            if abs(gz) < 1.0:      # 1 dps threshold
                gz = 0.0
            self._yaw_deg += gz * dt
            self.yaw.append(self._yaw_deg)
            self._yaw_deg += p["gz_dps"] * dt
            self.yaw.append(self._yaw_deg)
            self.roll.append(p["roll_deg"])
            self.pitch.append(p["pitch_deg"])

            self.ax.append(p["ax_g"])
            self.ay.append(p["ay_g"])
            self.az.append(p["az_g"])

            self.gx.append(p["gx_dps"])
            self.gy.append(p["gy_dps"])
            self.gz.append(p["gz_dps"])

        self.after(30, self._pump_imu)

    def _update_plot(self):
        if len(self.t) >= 2:
            xs = np.fromiter(self.t, float)

            self.l_roll.set_data(xs, np.fromiter(self.roll, float))
            self.l_pitch.set_data(xs, np.fromiter(self.pitch, float))
            self.l_yaw.set_data(xs, np.fromiter(self.yaw, float))

            self.l_ax.set_data(xs, np.fromiter(self.ax, float))
            self.l_ay.set_data(xs, np.fromiter(self.ay, float))
            self.l_az.set_data(xs, np.fromiter(self.az, float))

            self.l_gx.set_data(xs, np.fromiter(self.gx, float))
            self.l_gy.set_data(xs, np.fromiter(self.gy, float))
            self.l_gz.set_data(xs, np.fromiter(self.gz, float))

            for ax in (self.ax1, self.ax2, self.ax3):
                ax.relim()
                ax.autoscale_view()

            self.canvas.draw_idle()

        self.after(60, self._update_plot)

    def _update_statline(self):
        st = self.imu.status()
        p = self._last_pkt
        if p:
            self.status_var.set(
                f"[IMU] conn={st['connected']} buf={st['buf_len']} "
                f"seq={p['seq']} roll={p['roll_deg']:.2f} pitch={p['pitch_deg']:.2f} "
                f"gx={p['gx_dps']:.2f} gy={p['gy_dps']:.2f} gz={p['gz_dps']:.2f}"
            )
        else:
            self.status_var.set(
                f"[IMU] conn={st['connected']} buf={st['buf_len']} rx={st['rx_packets']} age={st['last_rx_age_s']:.2f}s"
            )



if __name__ == "__main__":
    ImuApp().mainloop()
