#!/usr/bin/env python3
"""
dashboard.py - Dual-Stage Grid Power Quality Improvement System (Final Patched)
Real-time dashboard for the STM32F401RE APFC + AHF (TB6612FNG) firmware.

    py dashboard.py                 # auto-detect ST-LINK / COM35
    py dashboard.py --port COM35    # explicit port
    py dashboard.py --demo          # built-in simulator, no hardware needed

Requires: numpy, matplotlib, pyserial  (py -m pip install numpy matplotlib pyserial)

Telemetry frames parsed: $W $M $H $P $A $K $R, committed atomically on $E.
Commands sent:  $C,<idx>,<mode>,<on>   capacitor AUTO/MANUAL override
                $A,<en>                AHF H-bridge ACTIVE(1) / STANDBY(0)
"""

import argparse
import math
import sys
import threading
import time

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Wedge
from matplotlib.widgets import Button

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # demo mode still works
    serial = None

# --------------------------------------------------------------------------- #
#  Protocol                                                                    #
# --------------------------------------------------------------------------- #
F_GRID = 50.0
CAP_UF = (2.5, 10.0, 20.0)
N_H = 15
M_KEYS = ("vrms", "irms", "p", "s", "q", "pf", "thd_v", "thd_i", "crest",
          "vpk", "ipk", "dpf", "dva")
P_KEYS = ("state", "cap", "pf", "q", "hold", "target", "c_need")
A_KEYS = ("active", "peak", "ref_peak", "depth", "adv")
K_KEYS = ("mask", "state", "ok", "bad", "rx")
R_KEYS = ("off_v", "min_v", "max_v", "clip_v", "off_i", "min_i", "max_i", "clip_i",
          "off_a", "min_a", "max_a", "clip_a", "adc_ovr", "win_drop", "tx_skip")

# --------------------------------------------------------------------------- #
#  Theme                                                                       #
# --------------------------------------------------------------------------- #
BG, PANEL, GRID, FG, MUTED = "#0d1117", "#161b22", "#30363d", "#c9d1d9", "#8b949e"
C_V, C_I, C_AHF, C_REF, C_COR = "#ff7b72", "#d2a8ff", "#00e5ff", "#ffd400", "#3fb950"
C_WARN, C_BAD, C_OFF, C_ON = "#f0883e", "#f85149", "#262c36", "#1f6f3a"


# =========================================================================== #
#  Thread-safe telemetry store                                                #
# =========================================================================== #
class Telemetry:
    """Lines build up a pending frame; '$E' commits it under the lock, so the
    GUI never sees half of one frame mixed with half of the next."""

    def __init__(self):
        self.lock = threading.Lock()
        self.wave = {}
        self.h = {"I": np.zeros(N_H), "A": np.zeros(N_H)}
        self.m = dict.fromkeys(M_KEYS, 0.0)
        self.pfc = dict.fromkeys(P_KEYS, 0.0)
        self.ahf = dict.fromkeys(A_KEYS, 0.0)
        self.k = dict.fromkeys(K_KEYS, 0.0)
        self.r = dict.fromkeys(R_KEYS, 0.0)
        self.frames = 0
        self.bad_lines = 0
        self.last_frame = 0.0
        self.fps = 0.0
        self._pend = self._new_pending()

    @staticmethod
    def _new_pending():
        return {"wave": {}, "h": {}, "m": {}, "pfc": {}, "ahf": {}, "k": {}, "r": {}}

    @staticmethod
    def _kv(keys, fields):
        return dict(zip(keys, (float(x) for x in fields)))

    def feed(self, line):
        line = line.strip()
        if not line.startswith("$"):
            return
        f = line[1:].split(",")
        tag, p = f[0], self._pend
        try:
            if tag == "W" and len(f) > 2:
                p["wave"][f[1]] = np.array(f[2:], dtype=float)
            elif tag == "H" and len(f) > 2:
                arr = np.zeros(N_H)
                vals = np.array(f[2:2 + N_H], dtype=float)
                arr[:vals.size] = vals
                p["h"][f[1]] = arr
            elif tag == "M":
                p["m"] = self._kv(M_KEYS, f[1:])
            elif tag == "P":
                p["pfc"] = self._kv(P_KEYS, f[1:])
            elif tag == "A":
                p["ahf"] = self._kv(A_KEYS, f[1:])
            elif tag == "K":
                p["k"] = self._kv(K_KEYS, f[1:])
            elif tag == "R":
                p["r"] = self._kv(R_KEYS, f[1:])
            elif tag == "E":
                self._commit()
        except ValueError:
            self.bad_lines += 1

    def _commit(self):
        p, self._pend = self._pend, self._new_pending()
        now = time.monotonic()
        with self.lock:
            self.wave.update(p["wave"])
            self.h.update(p["h"])
            for name in ("m", "pfc", "ahf", "k", "r"):
                getattr(self, name).update(p[name])
            if self.last_frame:
                dt = now - self.last_frame
                if dt > 0:
                    self.fps = 0.8 * self.fps + 0.2 / dt
            self.last_frame = now
            self.frames += 1

    def snapshot(self):
        with self.lock:
            return {
                "wave": dict(self.wave), "h": dict(self.h), "m": dict(self.m),
                "pfc": dict(self.pfc), "ahf": dict(self.ahf), "k": dict(self.k),
                "r": dict(self.r), "frames": self.frames, "bad": self.bad_lines,
                "age": time.monotonic() - self.last_frame if self.last_frame else 1e9,
                "fps": self.fps,
            }

    def optimistic(self, k=None, ahf=None):
        """Reflect a sent command at once; the next frame overwrites it with truth."""
        with self.lock:
            if k:
                self.k.update(k)
            if ahf:
                self.ahf.update(ahf)


# =========================================================================== #
#  Serial link (background reader, auto-reconnect)                            #
# =========================================================================== #
def find_port():
    if serial is None:
        return None
    ports = list(list_ports.comports())
    for p in ports:
        desc = "%s %s" % (p.description or "", p.manufacturer or "")
        if p.vid == 0x0483 or any(k in desc for k in ("STLink", "ST-Link", "STMicro", "USB Serial")):
            return p.device
    for p in ports:
        if p.device.upper() == "COM35":
            return p.device
    return ports[0].device if ports else None


class SerialLink(threading.Thread):
    def __init__(self, port, baud, telem):
        super().__init__(daemon=True)
        self.port, self.baud, self.telem = port, baud, telem
        self.ser = None
        self.wlock = threading.Lock()
        self.stop_evt = threading.Event()
        self.status = "connecting..."

    def run(self):
        while not self.stop_evt.is_set():
            port = self.port or find_port()
            if not port:
                self.status = "no serial port found"
                self.stop_evt.wait(1.0)
                continue
            try:
                with serial.Serial(port, self.baud, timeout=0.2) as ser:
                    ser.reset_input_buffer()
                    with self.wlock:
                        self.ser = ser
                    self.status = "LIVE  %s @ %d" % (port, self.baud)
                    buf = b""
                    while not self.stop_evt.is_set():
                        chunk = ser.read(ser.in_waiting or 1)
                        if not chunk:
                            continue
                        buf += chunk
                        *lines, buf = buf.split(b"\n")
                        for ln in lines:
                            self.telem.feed(ln.decode("ascii", "ignore"))
                        if len(buf) > 16384:
                            buf = b""
            except (serial.SerialException, OSError) as exc:
                self.status = "disconnected: %s" % (str(exc).splitlines()[0][:60] if str(exc) else "error")
            finally:
                with self.wlock:
                    self.ser = None
            self.stop_evt.wait(1.0)

    def send(self, cmd):
        with self.wlock:
            if self.ser is None:
                return False
            try:
                self.ser.write(cmd.encode("ascii"))
                return True
            except (serial.SerialException, OSError):
                return False

    def close(self):
        self.stop_evt.set()


# =========================================================================== #
#  Demo simulator - emits the firmware's exact line protocol                  #
# =========================================================================== #
class DemoLink(threading.Thread):
    N = 200

    def __init__(self, telem):
        super().__init__(daemon=True)
        self.telem = telem
        self.stop_evt = threading.Event()
        self.mtx = threading.Lock()
        self.status = "DEMO  (simulated grid)"
        self.lock_mask = self.lock_state = 0
        self.relays = self.target = 0
        self.off_t = [-1e9] * 3
        self.ahf_on = 1
        self.ok = self.bad = self.rx = 0
        self.hold, self.c_need = 1, 0.0
        self.t0 = time.monotonic()
        self.rng = np.random.RandomState(7)
        k = np.arange(self.N)
        h = np.arange(1, N_H + 1)[:, None]
        self.cos = np.cos(2 * np.pi * h * k / self.N)
        self.sin = np.sin(2 * np.pi * h * k / self.N)

    def send(self, cmd):
        with self.mtx:
            self.rx += len(cmd)
            f = cmd.strip()[1:].split(",")
            try:
                if f[0] == "C" and len(f) == 4:
                    i, mode, on = (int(x) for x in f[1:])
                    if not (0 <= i <= 2 and mode in (0, 1) and on in (0, 1)):
                        raise ValueError
                    bit = 1 << i
                    if mode:
                        self.lock_mask |= bit
                        self.lock_state = (self.lock_state | bit) if on else (self.lock_state & ~bit)
                    else:
                        self.lock_mask &= ~bit
                        self.lock_state &= ~bit
                    self.target = (self.target & ~self.lock_mask) | (self.lock_state & self.lock_mask)
                elif f[0] == "A" and len(f) == 2 and f[1] in ("0", "1"):
                    self.ahf_on = int(f[1])
                else:
                    raise ValueError
                self.ok += 1
            except ValueError:
                self.bad += 1
        return True

    def close(self):
        self.stop_evt.set()

    def run(self):
        while not self.stop_evt.wait(0.5):
            for line in self.frame().splitlines():
                self.telem.feed(line)

    @staticmethod
    def _cap(st):
        return sum(c for i, c in enumerate(CAP_UF) if st >> i & 1)

    def _dft(self, x):
        return 2.0 / self.N * (self.cos @ x), 2.0 / self.N * (self.sin @ x)

    def frame(self):
        with self.mtx:
            return self._frame()

    def _frame(self):
        n, rng, now = self.N, self.rng, time.monotonic()
        t = now - self.t0
        th = 2 * np.pi * np.arange(n) / n
        w = 2 * np.pi * F_GRID

        v = 230 * math.sqrt(2) * (np.sin(th) + 0.02 * np.sin(5 * th + 0.3)) + rng.normal(0, 0.8, n)
        lag = math.radians(40 + 12 * math.sin(2 * math.pi * t / 45))
        i_load = 2.0 * math.sqrt(2) * np.sin(th - lag)
        x = np.sin(th)
        i_smps = 1.6 * np.sign(x) * np.clip((np.abs(x) - 0.82) / 0.18, 0, 1) ** 1.5
        i_cap = w * self._cap(self.relays) * 1e-6 * 230 * math.sqrt(2) * np.cos(th)
        i = i_load + i_smps + i_cap + rng.normal(0, 0.01, n)

        vrms, irms = math.sqrt(np.mean(v * v)), math.sqrt(np.mean(i * i))
        p = float(np.mean(v * i))
        s = vrms * irms
        av, bv = self._dft(v)
        ai, bi = self._dft(i)
        p1 = 0.5 * (av[0] * ai[0] + bv[0] * bi[0])
        q1 = 0.5 * (av[0] * bi[0] - bv[0] * ai[0])
        dpf = p1 / math.hypot(p1, q1)
        dva = math.sqrt(max(s * s - p * p - q1 * q1, 0.0))
        hi = np.hypot(ai, bi) / math.sqrt(2)
        hv = np.hypot(av, bv) / math.sqrt(2)
        thd = lambda hh: 100 * math.sqrt(np.sum(hh[1:] ** 2)) / hh[0]

        i1 = ai[0] * np.cos(th) + bi[0] * np.sin(th)
        tref = -(i - i1)

        pk = np.max(np.abs(tref))
        if self.ahf_on and pk > 0.02:
            drive = 0.92 * tref / pk
            f = np.fft.rfftfreq(n, 1.0 / (n * F_GRID))
            a = 0.85 * np.fft.irfft(np.fft.rfft(drive) / (1 + 1j * f / 1500.0), n)
        else:
            a = np.zeros(n)
        a = a + rng.normal(0, 0.006, n)
        aa, ba = self._dft(a)
        ha = np.hypot(aa, ba) / math.sqrt(2)

        i_corr = i1 if self.ahf_on else i

        k_var = w * vrms * vrms * 1e-6
        cur = (self.target & ~self.lock_mask) | (self.lock_state & self.lock_mask)
        q_load = q1 + k_var * self._cap(self.relays)
        self.c_need = max((q_load - abs(p1) * math.sqrt(1 - 0.95 ** 2) / 0.95) / k_var, 0.0)
        combos = [st for st in range(8) if (st & self.lock_mask) == (self.lock_state & self.lock_mask)]
        best = min(combos, key=lambda st: (abs(self._cap(st) - self.c_need), bin(st ^ cur).count("1")))
        e_cur = abs(self._cap(cur) - self.c_need)
        if best != cur and e_cur > 1.25 and abs(self._cap(best) - self.c_need) < e_cur:
            self.target, self.hold = best, 0
        else:
            self.target, self.hold = cur, 1
        for idx in range(3):
            b = 1 << idx
            if self.target & b and not self.relays & b and now - self.off_t[idx] >= 0.5:
                self.relays |= b
            elif not self.target & b and self.relays & b:
                self.relays &= ~b
                self.off_t[idx] = now

        def wv(arr, dec):
            return ",".join("%.*f" % (dec, x) for x in arr[::2])

        def raw(sig, k):
            r = 2048 + sig / k
            return "%.1f,%d,%d,0" % (np.mean(r), int(np.min(r)), int(np.max(r)))

        k_i = (3.3 / 4095) / 0.185
        pf = p / s
        lines = [
            "$W,V," + wv(v, 1), "$W,I," + wv(i, 3), "$W,A," + wv(a, 3),
            "$W,T," + wv(tref, 3), "$W,C," + wv(i_corr, 3),
            "$M," + ",".join("%.4f" % x for x in (vrms, irms, p, s, q1, pf, thd(hv), thd(hi),
                                                  np.max(np.abs(i)) / irms, np.max(np.abs(v)),
                                                  np.max(np.abs(i)), dpf, dva)),
            "$H,I," + ",".join("%.4f" % x for x in hi),
            "$H,A," + ",".join("%.4f" % x for x in ha),
            "$P,%d,%.1f,%.4f,%.2f,%d,%d,%.2f" % (self.relays, self._cap(self.relays), pf, q1,
                                                 self.hold, self.target, self.c_need),
            "$A,%d,%.4f,%.4f,0.92,0" % (self.ahf_on, np.max(np.abs(a)), pk),
            "$K,%d,%d,%d,%d,%d" % (self.lock_mask, self.lock_state, self.ok, self.bad, self.rx),
            "$R," + raw(v, 3.3 / 4095 * 1014) + "," + raw(i, k_i) + "," + raw(a, k_i) + ",0,0,0",
            "$E",
        ]
        return "\n".join(lines) + "\n"


# =========================================================================== #
#  Dashboard                                                                  #
# =========================================================================== #
def _style(ax, title, ylabel=None):
    ax.set_facecolor(PANEL)
    ax.set_title(title, loc="left", fontsize=10, color=FG, fontweight="bold", pad=4)
    ax.grid(True, color=GRID, lw=0.6, alpha=0.8)
    ax.tick_params(colors=MUTED, labelsize=8)
    for sp in ax.spines.values():
        sp.set_color(GRID)
    if ylabel:
        ax.set_ylabel(ylabel, color=MUTED, fontsize=8.5)


def _corr(a, b):
    if a is None or b is None or a.size != b.size or a.size < 4:
        return float("nan")
    sa, sb = np.std(a), np.std(b)
    if sa < 1e-9 or sb < 1e-9:
        return float("nan")
    return float(np.mean((a - a.mean()) * (b - b.mean())) / (sa * sb))


class Dashboard:
    def __init__(self, telem, link):
        self.telem, self.link = telem, link
        self._lim = {}
        self._needle_pf = 0.0
        self._note, self._note_t = "", 0.0

        plt.style.use("dark_background")
        self.fig = fig = plt.figure(figsize=(17, 9.6), facecolor=BG)
        try:
            fig.canvas.manager.set_window_title("Grid Power Quality - APFC + AHF")
        except AttributeError:
            pass
        fig.text(0.045, 0.962, "DUAL-STAGE GRID POWER QUALITY SYSTEM", fontsize=15,
                 fontweight="bold", color=FG)
        fig.text(0.045, 0.938, "Stage 1  APFC switched capacitor bank    ·    "
                 "Stage 2  Active Harmonic Filter (TB6612FNG, 80 kHz unipolar)",
                 fontsize=9.5, color=MUTED)
        self.status_txt = fig.text(0.985, 0.955, "", ha="right", fontsize=10, fontweight="bold")
        self.note_txt = fig.text(0.985, 0.935, "", ha="right", fontsize=8.5, color=MUTED)

        outer = gridspec.GridSpec(1, 2, figure=fig, width_ratios=[2.6, 1], left=0.045,
                                  right=0.985, top=0.905, bottom=0.05, wspace=0.13)
        gl = gridspec.GridSpecFromSubplotSpec(5, 2, subplot_spec=outer[0], hspace=0.68,
                                              wspace=0.16, height_ratios=[1, 1, 1, 1, 1.4])
        gr = gridspec.GridSpecFromSubplotSpec(8, 1, subplot_spec=outer[1], hspace=0.45,
                                              height_ratios=[2.3, 1.9, 0.22, 0.5, 0.62, 0.22, 0.52, 1.6])

        # ---- waveforms (one 20 ms cycle) -------------------------------- #
        self.ax_v = fig.add_subplot(gl[0, :])
        self.ax_i = fig.add_subplot(gl[1, :], sharex=self.ax_v)
        self.ax_a = fig.add_subplot(gl[2, :], sharex=self.ax_v)
        self.ax_c = fig.add_subplot(gl[3, :], sharex=self.ax_v)
        _style(self.ax_v, "GRID VOLTAGE  V(t)", "V")
        _style(self.ax_i, "DISTORTED GRID CURRENT  I_grid(t)", "A")
        _style(self.ax_a, "AHF INJECTED CURRENT  I_ahf(t)  vs  TARGET REFERENCE", "I_ahf  [A]")
        _style(self.ax_c, "CORRECTED GRID CURRENT  (I_grid + compensation = H1 sine)", "A")
        self.ax_c.set_xlabel("time within one 50 Hz cycle  [ms]", color=MUTED, fontsize=8.5)
        self.ax_v.set_xlim(0, 1000.0 / F_GRID)

        self.l_v, = self.ax_v.plot([], [], color=C_V, lw=1.6)
        self.l_i, = self.ax_i.plot([], [], color=C_I, lw=1.6)
        self.l_a, = self.ax_a.plot([], [], color=C_AHF, lw=1.8, label="I_ahf measured (ACS712 #2)")
        self.ax_a2 = self.ax_a.twinx()
        self.ax_a2.tick_params(colors=C_REF, labelsize=8)
        self.ax_a2.set_ylabel("target  [A]", color=C_REF, fontsize=8.5)
        self.l_t, = self.ax_a2.plot([], [], color=C_REF, lw=1.4, ls="--", label="target  -(I - I₁)")
        self.ax_a.legend(handles=[self.l_a, self.l_t], loc="upper right", fontsize=7.5,
                         facecolor=PANEL, edgecolor=GRID, ncol=2)
        self.l_c0, = self.ax_c.plot([], [], color=MUTED, lw=0.9, alpha=0.45, label="before")
        self.l_c, = self.ax_c.plot([], [], color=C_COR, lw=2.0, label="after (H1 only)")
        self.ax_c.legend(loc="upper right", fontsize=7.5, facecolor=PANEL, edgecolor=GRID, ncol=2)
        for ax in (self.ax_v, self.ax_i, self.ax_a):
            plt.setp(ax.get_xticklabels(), visible=False)

        # ---- harmonic spectra ------------------------------------------- #
        hx = np.arange(1, N_H + 1)
        self.ax_hi = fig.add_subplot(gl[4, 0])
        self.ax_ha = fig.add_subplot(gl[4, 1])
        _style(self.ax_hi, "GRID CURRENT SPECTRUM", "% of I₁")
        _style(self.ax_ha, "AHF CURRENT SPECTRUM", "% of largest harmonic")
        self.b_hi = self.ax_hi.bar(hx, np.zeros(N_H), color=C_I, width=0.7)
        self.b_hi[0].set_color("#8c6bc8")
        self.b_ha = self.ax_ha.bar(hx, np.zeros(N_H), color=C_AHF, width=0.7, label="I_ahf measured")
        self.p_ref, = self.ax_ha.plot(hx[1:], np.zeros(N_H - 1), "D", color=C_REF, ms=5,
                                      label="required (grid harmonics)")
        self.ax_ha.legend(loc="upper right", fontsize=7.5, facecolor=PANEL, edgecolor=GRID)
        for ax in (self.ax_hi, self.ax_ha):
            ax.set_xticks(hx)
            ax.set_xlim(0.4, N_H + 0.6)
            ax.set_xlabel("harmonic order", color=MUTED, fontsize=8.5)
            ax.grid(True, axis="y", color=GRID, lw=0.6)
            ax.grid(False, axis="x")
        self.ax_ha.set_ylim(0, 125)

        # ---- right column ----------------------------------------------- #
        self._build_gauge(fig.add_subplot(gr[0]))
        self.ax_txt = fig.add_subplot(gr[1])
        self.ax_txt.set_facecolor(PANEL)
        self.ax_txt.set_xticks([]); self.ax_txt.set_yticks([])
        for sp in self.ax_txt.spines.values():
            sp.set_color(GRID)
        self.m_txt = self.ax_txt.text(0.05, 0.92, "", va="top", family="monospace",
                                      fontsize=9.2, color=FG, transform=self.ax_txt.transAxes)

        self._header(fig.add_subplot(gr[2]), "APFC  ·  CAPACITOR BANK  (target PF 0.95)")
        self.btn_auto = self._button(gr[3], "AUTO MODE", self._on_auto)
        gc = gridspec.GridSpecFromSubplotSpec(1, 3, subplot_spec=gr[4], wspace=0.12)
        self.btn_cap = [self._button(gc[j], "%g µF" % CAP_UF[j], self._cap_cb(j)) for j in range(3)]

        self._header(fig.add_subplot(gr[5]), "AHF  ·  TB6612FNG H-BRIDGE")
        ga = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=gr[6], wspace=0.12)
        self.btn_act = self._button(ga[0], "ACTIVE", lambda _e: self._ahf_cmd(1))
        self.btn_stb = self._button(ga[1], "STANDBY", lambda _e: self._ahf_cmd(0))

        self.ax_diag = fig.add_subplot(gr[7])
        self.ax_diag.set_facecolor(PANEL)
        self.ax_diag.set_xticks([]); self.ax_diag.set_yticks([])
        for sp in self.ax_diag.spines.values():
            sp.set_color(GRID)
        self.d_txt = self.ax_diag.text(0.04, 0.93, "", va="top", family="monospace",
                                       fontsize=8.0, color=MUTED, transform=self.ax_diag.transAxes)

        self.anim = FuncAnimation(fig, self.update, interval=150, cache_frame_data=False)

    # ---------------------------------------------------------------- UI --
    @staticmethod
    def _header(ax, text):
        ax.axis("off")
        ax.text(0.0, 0.2, text, fontsize=9.5, fontweight="bold", color=FG, transform=ax.transAxes)

    def _button(self, spec, label, cb):
        ax = self.fig.add_subplot(spec)
        b = Button(ax, label, color=C_OFF, hovercolor="#39414d")
        b.label.set_fontsize(9)
        b.label.set_color(FG)
        b.on_clicked(cb)
        for sp in ax.spines.values():
            sp.set_color(GRID)
        return b

    @staticmethod
    def _paint(btn, face, edge=GRID, lw=0.8):
        btn.color = face
        btn.hovercolor = face if face != C_OFF else "#39414d"
        btn.ax.set_facecolor(face)
        for sp in btn.ax.spines.values():
            sp.set_color(edge)
            sp.set_linewidth(lw)

    def _build_gauge(self, ax):
        ax.set_facecolor(BG)
        ax.set_xlim(-1.18, 1.18)
        ax.set_ylim(-0.48, 1.12)
        ax.set_aspect("equal")
        ax.axis("off")
        ax.set_title("POWER FACTOR", fontsize=10, color=FG, fontweight="bold", pad=2)
        for lo, hi, col in ((0.0, 0.70, C_BAD), (0.70, 0.90, C_WARN), (0.90, 1.0, C_COR)):
            ax.add_patch(Wedge((0, 0), 1.0, 180 - hi * 180, 180 - lo * 180, width=0.17,
                               facecolor=col, edgecolor=BG, lw=1.5, alpha=0.9))
        for v in np.linspace(0, 1, 11):
            a = math.pi * (1 - v)
            ax.plot([0.78 * math.cos(a), 0.82 * math.cos(a)],
                    [0.78 * math.sin(a), 0.82 * math.sin(a)], color=MUTED, lw=1)
            if round(v * 10) % 2 == 0:
                ax.text(0.66 * math.cos(a), 0.66 * math.sin(a), "%.1f" % v, ha="center",
                        va="center", fontsize=7.5, color=MUTED)
        a = math.pi * (1 - 0.95)
        ax.plot([0.8 * math.cos(a), 1.03 * math.cos(a)], [0.8 * math.sin(a), 1.03 * math.sin(a)],
                color=FG, lw=1.2, ls=":")
        self.needle, = ax.plot([0, -0.8], [0, 0], color=FG, lw=3, solid_capstyle="round")
        ax.add_patch(Circle((0, 0), 0.06, color=FG, zorder=5))
        self.dpf_mark, = ax.plot([], [], marker="v", color=C_AHF, ms=8, ls="")
        self.g_val = ax.text(0, -0.17, "--", ha="center", va="center", fontsize=24,
                             fontweight="bold", color=FG)
        self.g_sub = ax.text(0, -0.40, "", ha="center", va="center", fontsize=8.5, color=MUTED)

    def _say(self, text):
        self._note, self._note_t = text, time.monotonic()

    def _send(self, cmd):
        ok = self.link.send(cmd)
        self._say(("sent  " if ok else "NOT SENT (no link)  ") + cmd.strip())
        return ok

    # ----------------------------------------------------------- commands --
    def _locks(self):
        with self.telem.lock:
            return int(self.telem.k["mask"]), int(self.telem.k["state"]), int(self.telem.pfc["state"])

    def _on_auto(self, _evt):
        mask, _lst, st = self._locks()
        if mask == 0:   # AUTO -> MANUAL: freeze every relay where it is now
            for j in range(3):
                self._send("$C,%d,1,%d\n" % (j, st >> j & 1))
            self.telem.optimistic(k={"mask": 7, "state": st})
        else:           # MANUAL/partial -> AUTO: release all locks
            for j in range(3):
                self._send("$C,%d,0,0\n" % j)
            self.telem.optimistic(k={"mask": 0, "state": 0})

    def _cap_cb(self, j):
        def cb(_evt):
            mask, lst, st = self._locks()
            bit = 1 << j
            if not (mask & bit):        # AUTO -> immediately toggle current physical relay state
                mode = 1
                on = 0 if (st & bit) else 1
            elif lst & bit:             # MAN ON -> MAN OFF
                mode, on = 1, 0
            else:                       # MAN OFF -> release back to AUTO
                mode, on = 0, 0
            self._send("$C,%d,%d,%d\n" % (j, mode, on))
            mask = (mask | bit) if mode else (mask & ~bit)
            lst = (lst | bit) if on else (lst & ~bit)
            self.telem.optimistic(k={"mask": mask, "state": lst})
        return cb

    def _ahf_cmd(self, en):
        self._send("$A,%d\n" % en)
        self.telem.optimistic(ahf={"active": en})

    # ------------------------------------------------------------- update --
    def _yscale(self, ax, key, arrays, floor):
        m = max((float(np.max(np.abs(a))) for a in arrays if a is not None and a.size), default=0.0)
        tgt = max(m * 1.18, floor)
        cur = self._lim.get(key, tgt)
        cur = tgt if tgt > cur else 0.9 * cur + 0.1 * tgt
        self._lim[key] = cur
        ax.set_ylim(-cur, cur)

    def update(self, _frame=None):
        s = self.telem.snapshot()
        w, m, pfc, ahf, k, r = s["wave"], s["m"], s["pfc"], s["ahf"], s["k"], s["r"]

        live = s["age"] < 2.5
        self.status_txt.set_text("● " + (self.link.status if live else "NO DATA  ·  " + self.link.status))
        self.status_txt.set_color(C_COR if live else C_BAD)
        note = self._note if time.monotonic() - self._note_t < 4 else ""
        self.note_txt.set_text("frames %d  ·  %.1f fps  ·  bad lines %d    %s"
                               % (s["frames"], s["fps"], s["bad"], note))
        if not s["frames"]:
            return []

        # waveforms
        def put(line, key):
            y = w.get(key)
            if y is None or not y.size:
                line.set_data([], [])
                return None
            line.set_data(np.arange(y.size) * (1000.0 / F_GRID / y.size), y)
            return y

        yv = put(self.l_v, "V")
        yi = put(self.l_i, "I")
        ya = put(self.l_a, "A")
        yt = put(self.l_t, "T")
        yc = put(self.l_c, "C")
        put(self.l_c0, "I")
        self._yscale(self.ax_v, "v", [yv], 10.0)
        self._yscale(self.ax_i, "i", [yi], 0.05)
        self._yscale(self.ax_a, "a", [ya], 0.05)
        self._yscale(self.ax_a2, "t", [yt], 0.02)
        self._yscale(self.ax_c, "c", [yi, yc], 0.05)
        rho = _corr(ya, yt)
        active = int(ahf["active"])
        self.ax_a.set_title("AHF INJECTED CURRENT  I_ahf(t)  vs  TARGET REFERENCE    "
                            "[%s   ρ = %s]" % ("ACTIVE" if active else "STANDBY",
                                                    "--" if math.isnan(rho) else "%.3f" % rho),
                            loc="left", fontsize=10, color=FG, fontweight="bold", pad=4)
        self.ax_i.set_title("DISTORTED GRID CURRENT  I_grid(t)    [THD %.1f %%   crest %.2f]"
                            % (m["thd_i"], m["crest"]), loc="left", fontsize=10, color=FG,
                            fontweight="bold", pad=4)
        self.ax_v.set_title("GRID VOLTAGE  V(t)    [%.1f Vrms   THD %.1f %%]" % (m["vrms"], m["thd_v"]),
                            loc="left", fontsize=10, color=FG, fontweight="bold", pad=4)

        # spectra
        hi, ha = s["h"]["I"], s["h"]["A"]
        hi_pct = 100.0 * hi / hi[0] if hi[0] > 1e-6 else np.zeros(N_H)
        for b, v in zip(self.b_hi, hi_pct):
            b.set_height(v)
        self.ax_hi.set_ylim(0, max(105.0, float(np.max(hi_pct)) * 1.1))
        self.ax_hi.set_title("GRID CURRENT SPECTRUM    [I₁ = %.3f A rms]" % hi[0], loc="left",
                             fontsize=10, color=FG, fontweight="bold", pad=4)
        ref_max, ahf_max = float(np.max(hi[1:])), float(np.max(ha[1:]))
        ref_n = 100.0 * hi[1:] / ref_max if ref_max > 1e-6 else np.zeros(N_H - 1)
        ahf_n = 100.0 * ha / ahf_max if ahf_max > 1e-6 else np.zeros(N_H)
        for b, v in zip(self.b_ha, ahf_n):
            b.set_height(min(v, 122.0))
        self.p_ref.set_ydata(ref_n)
        den = np.linalg.norm(ahf_n[1:]) * np.linalg.norm(ref_n)
        match = 100.0 * float(np.dot(ahf_n[1:], ref_n)) / den if den > 1e-9 else float("nan")
        self.ax_ha.set_title("AHF CURRENT SPECTRUM    [shape match %s   H1 leak %s]"
                             % ("--" if math.isnan(match) else "%.0f %%" % match,
                                "%.0f %%" % ahf_n[0] if ahf_max > 1e-6 else "--"),
                             loc="left", fontsize=10, color=FG, fontweight="bold", pad=4)

        # gauge (needle eased toward the reading)
        pf = min(abs(m["pf"]), 1.0)
        self._needle_pf += 0.35 * (pf - self._needle_pf)
        a = math.pi * (1 - self._needle_pf)
        self.needle.set_data([0, 0.8 * math.cos(a)], [0, 0.8 * math.sin(a)])
        ad = math.pi * (1 - min(abs(m["dpf"]), 1.0))
        self.dpf_mark.set_data([1.08 * math.cos(ad)], [1.08 * math.sin(ad)])
        self.g_val.set_text("%.3f" % pf)
        self.g_val.set_color(C_COR if pf >= 0.9 else C_WARN if pf >= 0.7 else C_BAD)
        kind = "LAGGING" if m["q"] > 0 else "LEADING" if m["q"] < 0 else ""
        self.g_sub.set_text("%s    ▼ DPF %.3f" % (kind, m["dpf"]))

        self.m_txt.set_text(
            "Vrms  %7.1f V      Irms %7.3f A\n"
            "P     %7.1f W      S    %7.1f VA\n"
            "Q1    %+7.1f var    D    %7.1f VA\n"
            "PF    %7.3f        DPF  %7.3f\n"
            "THD-I %7.1f %%      THD-V %6.1f %%\n"
            "Vpk   %7.1f V      Ipk  %7.3f A\n"
            "Crest %7.2f        Caps %7.1f µF\n"
            "C_req %7.2f µF     APFC %s"
            % (m["vrms"], m["irms"], m["p"], m["s"], m["q"], m["dva"], m["pf"], m["dpf"],
               m["thd_i"], m["thd_v"], m["vpk"], m["ipk"], m["crest"], pfc["cap"],
               pfc["c_need"], "HOLD" if int(pfc["hold"]) else "SWITCHING"))

        # APFC buttons reflect the firmware's reported state
        mask, lst = int(k["mask"]), int(k["state"])
        st, tgt = int(pfc["state"]), int(pfc["target"])
        if mask == 0:
            self.btn_auto.label.set_text("AUTO MODE:  ON")
            self._paint(self.btn_auto, C_ON)
        else:
            self.btn_auto.label.set_text("AUTO MODE:  %s" % ("OFF (MANUAL)" if mask == 7 else "PARTIAL"))
            self._paint(self.btn_auto, "#5a3a12", C_WARN, 1.6)
        for j, b in enumerate(self.btn_cap):
            on, locked, want = st >> j & 1, mask >> j & 1, tgt >> j & 1
            state = "ON" if on else "OFF"
            if want != on:
                state += "→ON" if want else "→OFF"
            b.label.set_text("%g µF\n%s · %s" % (CAP_UF[j], "MAN" if locked else "AUTO", state))
            self._paint(b, C_ON if on else C_OFF, C_WARN if locked else GRID, 2.0 if locked else 0.8)

        self._paint(self.btn_act, "#0b4f5c" if active else C_OFF, C_AHF if active else GRID,
                    1.8 if active else 0.8)
        self._paint(self.btn_stb, "#4a3410" if not active else C_OFF, C_WARN if not active else GRID,
                    1.8 if not active else 0.8)

        clip = int(r["clip_v"] + r["clip_i"] + r["clip_a"])
        self.d_txt.set_text(
            "ADC     offset    min   max  clip\n"
            "  V   %7.1f  %5d %5d %5d\n"
            "  I   %7.1f  %5d %5d %5d\n"
            "  A   %7.1f  %5d %5d %5d\n"
            "ovr %d  drop %d  tx-skip %d  %s\n"
            "UART  cmd ok %d  bad %d  rx %d B\n"
            "AHF   depth %.2f  adv %d  pk %.3f/%.3f A"
            % (r["off_v"], r["min_v"], r["max_v"], r["clip_v"],
               r["off_i"], r["min_i"], r["max_i"], r["clip_i"],
               r["off_a"], r["min_a"], r["max_a"], r["clip_a"],
               r["adc_ovr"], r["win_drop"], r["tx_skip"], "CLIPPING!" if clip else "",
               k["ok"], k["bad"], k["rx"],
               ahf["depth"], ahf["adv"], ahf["peak"], ahf["ref_peak"]))
        self.d_txt.set_color(C_BAD if clip else MUTED)
        return []


# =========================================================================== #
def main():
    ap = argparse.ArgumentParser(description="APFC + AHF real-time dashboard")
    ap.add_argument("--port", help="serial port, e.g. COM35 or /dev/ttyACM0 (default: auto)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true", help="run against the built-in simulator")
    args = ap.parse_args()

    if not args.demo and serial is None:
        sys.exit("pyserial is not installed:  py -m pip install pyserial   (or run with --demo)")

    telem = Telemetry()
    link = DemoLink(telem) if args.demo else SerialLink(args.port, args.baud, telem)
    link.start()
    Dashboard(telem, link)
    try:
        plt.show()
    finally:
        link.close()


if __name__ == "__main__":
    main()