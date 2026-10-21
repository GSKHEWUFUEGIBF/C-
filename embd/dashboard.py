#!/usr/bin/env python3
"""
dashboard.py - Dual-Stage Grid Power Quality Improvement System
Real-time dashboard for the STM32F401RE APFC + AHF (TB6612FNG) firmware.

Two tabs share the grid voltage and grid current waveforms on top:
    * Power Factor Correction  - PF gauge, power metrics, capacitor bank, trends
    * Harmonic Mitigation      - AHF tracking, corrected current, spectra, diagnostics

    py dashboard.py                 # auto-detect ST-LINK / COM35
    py dashboard.py --port COM35    # explicit port
    py dashboard.py --demo          # built-in simulator, no hardware needed

Requires: numpy, pyserial, pyqtgraph, PySide6
          python -m pip install numpy pyserial pyqtgraph PySide6

Telemetry frames parsed: $W $M $H $P $A $K $R, committed atomically on $E.
Commands sent:  $C,<idx>,<mode>,<on>   capacitor AUTO/MANUAL override
                $A,<en>                AHF H-bridge ACTIVE(1) / STANDBY(0)
"""

import argparse
import collections
import math
import sys
import threading
import time

try:
    import numpy as np
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtGui, QtWidgets
except ImportError as exc:
    sys.exit("Missing library (%s). Install with:\n"
             "    %s -m pip install numpy pyserial pyqtgraph PySide6" % (exc.name, sys.executable))

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
CAP_RECLOSE_S = 3.0          # firmware CAP_RECLOSE_MS: discharge guard before a cap re-closes
N_H = 15
M_KEYS = ("vrms", "irms", "p", "s", "q", "pf", "thd_v", "thd_i", "crest",
          "vpk", "ipk", "dpf", "dva", "freq")
P_KEYS = ("state", "cap", "pf", "q", "hold", "target", "c_need")
A_KEYS = ("active", "peak", "ref_peak", "duty", "resid")   # duty: table peak, resid: loop error %
K_KEYS = ("mask", "state", "ok", "bad", "rx")
R_KEYS = ("off_v", "min_v", "max_v", "clip_v", "off_i", "min_i", "max_i", "clip_i",
          "off_a", "min_a", "max_a", "clip_a", "adc_ovr", "win_drop", "tx_skip")
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
        if self.ahf_on and pk > 0.02:     # converged selective-harmonic loop: h2..h15 only
            f = np.fft.rfftfreq(n, 1.0 / (n * F_GRID))
            a = np.fft.irfft(np.fft.rfft(tref) * np.where((f > 75) & (f < 775), 0.97, 0.0), n)
            duty = min(0.92, 0.35 * np.max(np.abs(a)))
        else:
            a, duty = np.zeros(n), 0.0
        a = a + rng.normal(0, 0.006, n)
        aa, ba = self._dft(a)
        ha = np.hypot(aa, ba) / math.sqrt(2)
        resid = 100.0 * math.sqrt(np.sum(np.hypot(ai[1:] + aa[1:], bi[1:] + ba[1:]) ** 2)
                                  / max(np.sum(np.hypot(ai[1:], bi[1:]) ** 2), 1e-12))

        i_corr = i + a                    # firmware sends I + A (measured)

        k_var = w * vrms * vrms * 1e-6
        cur = (self.target & ~self.lock_mask) | (self.lock_state & self.lock_mask)
        q_load = q1 + k_var * self._cap(self.relays)
        self.c_need = max((q_load - abs(p1) * math.sqrt(1 - 0.95 ** 2) / 0.95) / k_var, 0.0)
        if self.lock_mask == 7:     # all manual: firmware reports the locked bank as "required"
            self.c_need = self._cap(self.lock_state)
        combos = [st for st in range(8) if (st & self.lock_mask) == (self.lock_state & self.lock_mask)]
        best = min(combos, key=lambda st: (abs(self._cap(st) - self.c_need), bin(st ^ cur).count("1")))
        e_cur = abs(self._cap(cur) - self.c_need)
        if best != cur and e_cur > 1.25 and abs(self._cap(best) - self.c_need) < e_cur:
            self.target, self.hold = best, 0
        else:
            self.target, self.hold = cur, 1
        for idx in range(3):
            b = 1 << idx
            if self.target & b and not self.relays & b and now - self.off_t[idx] >= CAP_RECLOSE_S:
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
                                                  np.max(np.abs(i)), dpf, dva,
                                                  F_GRID + 0.03 * math.sin(t / 20))),
            "$H,I," + ",".join("%.4f" % x for x in hi),
            "$H,A," + ",".join("%.4f" % x for x in ha),
            "$P,%d,%.1f,%.4f,%.2f,%d,%d,%.2f" % (self.relays, self._cap(self.relays), pf, q1,
                                                 self.hold, self.target, self.c_need),
            "$A,%d,%.4f,%.4f,%.3f,%.1f" % (self.ahf_on, np.max(np.abs(a)), pk, duty,
                                            resid if self.ahf_on else 0.0),
            "$K,%d,%d,%d,%d,%d" % (self.lock_mask, self.lock_state, self.ok, self.bad, self.rx),
            "$R," + raw(v, 3.3 / 4095 * 1014) + "," + raw(i, k_i) + "," + raw(a, k_i) + ",0,0,0",
            "$E",
        ]
        return "\n".join(lines) + "\n"


def _corr(a, b):
    if a is None or b is None or a.size != b.size or a.size < 4:
        return float("nan")
    sa, sb = np.std(a), np.std(b)
    if sa < 1e-9 or sb < 1e-9:
        return float("nan")
    return float(np.mean((a - a.mean()) * (b - b.mean())) / (sa * sb))



def _spectrum(y):
    """RMS magnitude of harmonics 1..N_H of one cycle of samples."""
    out = np.zeros(N_H)
    if y is None or y.size < 2 * N_H + 1:
        return out
    x = np.fft.rfft(y) * 2.0 / y.size / math.sqrt(2)
    mag = np.abs(x[1:N_H + 1])
    out[:mag.size] = mag
    return out


def _thd(h):
    return 100.0 * math.sqrt(float(np.sum(h[1:] ** 2))) / h[0] if h[0] > 1e-9 else float("nan")


# =========================================================================== #
#  Theme (light)                                                              #
# =========================================================================== #
BG, CARD, BORDER = "#f4f5f7", "#ffffff", "#e4e6eb"
FG, FG2, MUTED, GRID_LINE = "#1f2328", "#4b5563", "#8b919a", "#eceef1"
C_V, C_I = "#2a78d6", "#eb6834"                     # voltage, grid current
C_AHF, C_REF = "#1baf7a", "#6250d6"                 # AHF measured, target reference
C_BEFORE, C_AFTER = "#a3a8b0", "#1baf7a"            # uncorrected, corrected
C_PF, C_DPF = "#2a78d6", "#6250d6"
C_GOOD, C_WARN, C_BAD = "#1a9e6a", "#c98500", "#d63b3a"
HIST_S = 120.0                                      # trend window, seconds

QSS = """
QWidget { color: %(FG)s; font-family: "Segoe UI", "Inter", "Helvetica Neue", sans-serif; font-size: 10pt; }
#root, QScrollArea, QScrollArea > QWidget > QWidget { background: %(BG)s; }
QFrame#card { background: %(CARD)s; border: 1px solid %(BORDER)s; border-radius: 12px; }
QFrame#card QLabel { background: transparent; }
QLabel#title    { font-size: 16pt; font-weight: 700; }
QLabel#subtitle { color: %(MUTED)s; font-size: 9.5pt; }
QLabel#cardTitle{ font-size: 10.5pt; font-weight: 650; }
QLabel#cardSub  { color: %(MUTED)s; font-size: 9pt; }
QLabel#statLbl  { color: %(FG2)s; font-size: 9pt; }
QLabel#statVal  { font-size: 17pt; font-weight: 650; }
QLabel#statSub  { color: %(MUTED)s; font-size: 8.5pt; }
QLabel#mono     { font-family: "Cascadia Mono", "Consolas", monospace; font-size: 9pt; color: %(FG2)s; }
QLabel#section  { color: %(MUTED)s; font-size: 8.5pt; font-weight: 700; letter-spacing: 1px; }
QTabWidget::pane { border: 0; }
QTabBar { qproperty-drawBase: 0; }
QTabBar::tab { background: transparent; color: %(FG2)s; padding: 9px 22px; margin: 0 4px 10px 0;
               border: 1px solid transparent; border-radius: 9px; font-weight: 600; font-size: 10.5pt; }
QTabBar::tab:hover    { background: #eceef1; }
QTabBar::tab:selected { background: %(CARD)s; color: %(FG)s; border: 1px solid %(BORDER)s; }
QPushButton { background: %(CARD)s; border: 1px solid #d6d9de; border-radius: 9px; padding: 8px 10px; font-weight: 600; }
QPushButton:hover { background: #f0f2f5; }
QPushButton:pressed { background: #e6e9ed; }
QScrollBar:vertical { background: transparent; width: 10px; margin: 2px; }
QScrollBar::handle:vertical { background: #cfd3d9; border-radius: 4px; min-height: 30px; }
QScrollBar::add-line, QScrollBar::sub-line { height: 0; }
""" % dict(FG=FG, FG2=FG2, BG=BG, CARD=CARD, BORDER=BORDER, MUTED=MUTED)


def _css(widget, css):
    """setStyleSheet re-polishes the widget, so only call it when the style really changes."""
    if getattr(widget, "_css", None) != css:
        widget._css = css
        widget.setStyleSheet(css)


def _btn_style(face, edge, fg=FG, width=1):
    return ("QPushButton { background: %s; border: %dpx solid %s; color: %s; }"
            "QPushButton:hover { border-color: %s; }" % (face, width, edge, fg, fg))


# =========================================================================== #
#  Widgets                                                                    #
# =========================================================================== #
class Card(QtWidgets.QFrame):
    def __init__(self, title="", sub=""):
        super().__init__()
        self.setObjectName("card")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(16, 12, 16, 14)
        lay.setSpacing(8)
        if title:
            head = QtWidgets.QHBoxLayout()
            head.setSpacing(10)
            self.title = QtWidgets.QLabel(title, objectName="cardTitle")
            self.sub = QtWidgets.QLabel(sub, objectName="cardSub")
            head.addWidget(self.title)
            head.addWidget(self.sub)
            head.addStretch(1)
            self.head = head
            lay.addLayout(head)
        self.body = lay


class Stat(QtWidgets.QFrame):
    """Label / big value / small caption tile."""

    def __init__(self, label, sub=""):
        super().__init__()
        self.setObjectName("card")
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(14, 10, 14, 10)
        lay.setSpacing(1)
        self.lbl = QtWidgets.QLabel(label, objectName="statLbl")
        self.val = QtWidgets.QLabel("--", objectName="statVal")
        self.sub = QtWidgets.QLabel(sub, objectName="statSub")
        for w in (self.lbl, self.val, self.sub):
            lay.addWidget(w)
        self._last = None

    def set(self, value, unit="", sub=None, color=FG):
        key = (value, unit, sub, color)
        if key == self._last:          # avoid needless relayout
            return
        self._last = key
        self.val.setText('%s<span style="font-size:10pt;color:%s;font-weight:500">&nbsp;%s</span>'
                         % (value, MUTED, unit) if unit else value)
        _css(self.val, "color:%s" % color)
        if sub is not None:
            self.sub.setText(sub)


class Pill(QtWidgets.QLabel):
    def set(self, text, color):
        self.setText("●  " + text)
        _css(self, "QLabel { color:%s; background:%s; border:1px solid %s; border-radius:12px;"
                           " padding:5px 12px; font-weight:600; }" % (color, CARD, BORDER))


class PFGauge(QtWidgets.QWidget):
    """Semicircular PF gauge, painted directly; needle eases toward the reading."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(260, 230)
        self.pf = self.shown = 0.0
        self.dpf = 0.0
        self.sub = ""

    def set(self, pf, dpf, sub):
        self.pf, self.dpf, self.sub = min(abs(pf), 1.0), min(abs(dpf), 1.0), sub

    def step(self):
        d = self.pf - self.shown
        if abs(d) > 1e-4:
            self.shown += 0.18 * d
            self.update()

    def paintEvent(self, _evt):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        r = max(40.0, min(w / 2 - 24, h - 86))
        cx, cy = w / 2, 18 + r
        rect = QtCore.QRectF(cx - r, cy - r, 2 * r, 2 * r)
        thick = max(10.0, r * 0.11)

        def arc(lo, hi, color, width):
            pen = QtGui.QPen(QtGui.QColor(color), width)
            pen.setCapStyle(QtCore.Qt.PenCapStyle.FlatCap)
            p.setPen(pen)
            p.drawArc(rect, int((180 - hi * 180) * 16), int((hi - lo) * 180 * 16))

        for lo, hi, col in ((0.0, 0.70, "#f6d4d3"), (0.70, 0.90, "#f5e6c4"), (0.90, 1.0, "#cdeedf")):
            arc(lo, hi, col, thick)
        col = C_GOOD if self.pf >= 0.9 else C_WARN if self.pf >= 0.7 else C_BAD
        arc(0.0, self.shown, col, thick * 0.38)

        p.setFont(QtGui.QFont(self.font().family(), 8))
        for k in range(11):
            v = k / 10.0
            a = math.pi * (1 - v)
            r0, r1 = r - thick * 0.75, r - thick * 0.75 - (7 if k % 2 == 0 else 4)
            p.setPen(QtGui.QPen(QtGui.QColor(MUTED), 1.2))
            p.drawLine(QtCore.QPointF(cx + r0 * math.cos(a), cy - r0 * math.sin(a)),
                       QtCore.QPointF(cx + r1 * math.cos(a), cy - r1 * math.sin(a)))
            if k % 2 == 0:
                rl = r1 - 13
                p.drawText(QtCore.QRectF(cx + rl * math.cos(a) - 16, cy - rl * math.sin(a) - 8, 32, 16),
                           QtCore.Qt.AlignmentFlag.AlignCenter, "%.1f" % v)
        # 0.95 target tick
        a = math.pi * 0.05
        pen = QtGui.QPen(QtGui.QColor(FG), 1.4, QtCore.Qt.PenStyle.DotLine)
        p.setPen(pen)
        p.drawLine(QtCore.QPointF(cx + (r - thick) * math.cos(a), cy - (r - thick) * math.sin(a)),
                   QtCore.QPointF(cx + (r + 6) * math.cos(a), cy - (r + 6) * math.sin(a)))
        # DPF marker
        a = math.pi * (1 - self.dpf)
        rm = r + thick / 2 + 7
        tip = QtCore.QPointF(cx + (rm - 6) * math.cos(a), cy - (rm - 6) * math.sin(a))
        b1 = QtCore.QPointF(cx + (rm + 4) * math.cos(a + 0.035), cy - (rm + 4) * math.sin(a + 0.035))
        b2 = QtCore.QPointF(cx + (rm + 4) * math.cos(a - 0.035), cy - (rm + 4) * math.sin(a - 0.035))
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(C_DPF))
        p.drawPolygon(QtGui.QPolygonF([tip, b1, b2]))
        # needle
        a = math.pi * (1 - self.shown)
        ln = r - thick - 18
        pen = QtGui.QPen(QtGui.QColor(FG), 3.2)
        pen.setCapStyle(QtCore.Qt.PenCapStyle.RoundCap)
        p.setPen(pen)
        p.drawLine(QtCore.QPointF(cx, cy), QtCore.QPointF(cx + ln * math.cos(a), cy - ln * math.sin(a)))
        p.setPen(QtCore.Qt.PenStyle.NoPen)
        p.setBrush(QtGui.QColor(FG))
        p.drawEllipse(QtCore.QPointF(cx, cy), 6, 6)
        # readout
        f = QtGui.QFont(self.font().family(), 22)
        f.setWeight(QtGui.QFont.Weight.Bold)
        p.setFont(f)
        p.setPen(QtGui.QColor(col))
        p.drawText(QtCore.QRectF(0, cy + 8, w, 34), QtCore.Qt.AlignmentFlag.AlignCenter, "%.3f" % self.pf)
        p.setFont(QtGui.QFont(self.font().family(), 9))
        p.setPen(QtGui.QColor(FG2))
        p.drawText(QtCore.QRectF(0, cy + 42, w, 18), QtCore.Qt.AlignmentFlag.AlignCenter, self.sub)
        p.end()


class _Grid(pg.GraphicsObject):
    """Grid lines at the axis ticks. pyqtgraph's built-in grid replays a large
    QPicture on every repaint and was ~85 % of the paint time; this draws a few lines."""

    def __init__(self, pi):
        super().__init__()
        self.pi, self.x, self.y = pi, True, True
        self.pen = pg.mkPen(GRID_LINE, width=1)
        self.pen.setCosmetic(True)
        self.setZValue(-100)
        pi.addItem(self, ignoreBounds=True)
        pi.vb.sigRangeChanged.connect(self._changed)
        pi.vb.sigResized.connect(self._changed)

    def _changed(self, *_):
        self.prepareGeometryChange()
        self.update()

    def boundingRect(self):
        return self.pi.vb.viewRect()

    def paint(self, p, *_):
        vb = self.pi.vb
        (x0, x1), (y0, y1) = vb.viewRange()
        p.setPen(self.pen)
        if self.y:
            for v in self._level("left", y0, y1, vb.height(), 10):
                p.drawLine(QtCore.QPointF(x0, v), QtCore.QPointF(x1, v))
        if self.x:
            for v in self._level("bottom", x0, x1, vb.width(), 24):
                p.drawLine(QtCore.QPointF(v, y0), QtCore.QPointF(v, y1))

    def _level(self, side, lo, hi, px, most):
        """Finest tick level with at most `most` lines (minor ticks make it too busy)."""
        best = []
        for _sp, vals in self.pi.getAxis(side).tickValues(lo, hi, px):
            if len(vals) <= most:
                best = vals
        return best


class Plot(pg.PlotWidget):
    """Light, non-interactive plot with an optional hover readout."""

    def __init__(self, ylabel="", xlabel="", height=230):
        super().__init__()
        self.setMinimumHeight(height)
        self.setMaximumHeight(460)
        pi = self.getPlotItem()
        pi.hideButtons()
        pi.setMenuEnabled(False)
        self.setMouseEnabled(False, False)
        font = QtGui.QFont()
        font.setPointSizeF(8.5)
        for side in ("left", "bottom"):
            ax = pi.getAxis(side)
            ax.setPen(pg.mkPen(GRID_LINE))
            ax.setTextPen(pg.mkPen(MUTED))
            ax.setStyle(tickFont=font, tickLength=-4)
        if ylabel:
            pi.setLabel("left", ylabel, color=MUTED, size="8.5pt")
        if xlabel:
            pi.setLabel("bottom", xlabel, color=MUTED, size="8.5pt")
        pi.getAxis("left").setWidth(48)
        self.grid = _Grid(pi)
        self._hover = None

    def legend(self):
        lg = self.getPlotItem().addLegend(offset=(-8, 6), labelTextColor=FG2, labelTextSize="8.5pt",
                                          brush=pg.mkBrush(255, 255, 255, 225), pen=pg.mkPen(BORDER))
        lg.setColumnCount(3)
        return lg

    def line(self, color, width=2.0, name=None, dash=False):
        pen = pg.mkPen(color, width=width)
        if dash:
            pen.setStyle(QtCore.Qt.PenStyle.DashLine)
        return self.plot([], [], pen=pen, name=name)

    def hover(self, curves, fmt_x):
        """curves: [(curve, label, fmt)] - nearest-sample readout under the cursor."""
        vl = pg.InfiniteLine(angle=90, pen=pg.mkPen("#b8bdc5", width=1))
        tx = pg.TextItem(anchor=(0, 0), fill=pg.mkBrush(255, 255, 255, 235), border=pg.mkPen(BORDER))
        dots = pg.ScatterPlotItem(size=8, pen=pg.mkPen("w", width=1.5))
        for it in (vl, tx, dots):
            it.setZValue(50)
            it.hide()
            self.addItem(it, ignoreBounds=True)
        self._hover = (vl, tx, dots, curves, fmt_x)
        self.scene().sigMouseMoved.connect(self._on_move)

    def _on_move(self, pos):
        vl, tx, dots, curves, fmt_x = self._hover
        vb = self.getPlotItem().vb
        if not vb.sceneBoundingRect().contains(pos):
            return self.leaveEvent(None)
        x = vb.mapSceneToView(pos).x()
        rows, pts = [], []
        for c, label, fmt in curves:
            xs, ys = c.getData()
            if xs is None or not len(xs) or not c.isVisible():
                continue
            i = int(np.clip(np.searchsorted(xs, x), 0, len(xs) - 1))
            if i and abs(xs[i - 1] - x) < abs(xs[i] - x):
                i -= 1
            col = c.opts["pen"].color().name()
            pts.append({"pos": (xs[i], ys[i]), "brush": pg.mkBrush(col)})
            rows.append('<span style="color:%s">■</span> %s&nbsp;&nbsp;<b>%s</b>' % (col, label, fmt(ys[i])))
            x_snap = xs[i]
        if not rows:
            return
        vl.setPos(x_snap)
        dots.setData(pts)
        (x0, x1), (y0, y1) = vb.viewRange()
        tx.setHtml('<div style="color:%s;font-size:8.5pt">%s<br>%s</div>'
                   % (FG, '<span style="color:%s">%s</span>' % (MUTED, fmt_x(x_snap)), "<br>".join(rows)))
        tx.setAnchor((1.05, 0) if x_snap > (x0 + x1) / 2 else (-0.05, 0))
        tx.setPos(x_snap, y1)
        for it in (vl, tx, dots):
            it.show()

    def leaveEvent(self, ev):
        if self._hover:
            for it in self._hover[:3]:
                it.hide()
        if ev is not None:
            super().leaveEvent(ev)


def _grid(widgets, cols, spacing=12):
    g = QtWidgets.QGridLayout()
    g.setSpacing(spacing)
    for n, w in enumerate(widgets):
        g.addWidget(w, n // cols, n % cols)
    return g


def _row(*items, stretch=None, spacing=14):
    lay = QtWidgets.QHBoxLayout()
    lay.setSpacing(spacing)
    for n, w in enumerate(items):
        if isinstance(w, QtWidgets.QLayout):
            lay.addLayout(w, (stretch or [1] * len(items))[n])
        else:
            lay.addWidget(w, (stretch or [1] * len(items))[n])
    return lay


# =========================================================================== #
#  Dashboard                                                                  #
# =========================================================================== #
class Dashboard(QtWidgets.QMainWindow):
    def __init__(self, telem, link):
        super().__init__()
        self.telem, self.link = telem, link
        self._lim = {}
        self._frames_seen = -1
        self._note, self._note_t = "", 0.0
        self._last_status = 0.0
        self._t0 = time.monotonic()
        self.hist = collections.deque(maxlen=4000)   # (t, pf, dpf, cap, c_need, thd_before, thd_after)

        self.setWindowTitle("Grid Power Quality - APFC + AHF")
        self.resize(1560, 980)
        self.setStyleSheet(QSS)

        root = QtWidgets.QWidget(objectName="root")
        page = QtWidgets.QVBoxLayout(root)
        page.setContentsMargins(22, 16, 22, 18)
        page.setSpacing(14)
        page.addLayout(self._build_header())
        page.addLayout(self._build_common(), 2)

        self.tabs = QtWidgets.QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(self._build_pfc(), "Power Factor Correction")
        self.tabs.addTab(self._build_hm(), "Harmonic Mitigation")
        self.tabs.currentChanged.connect(lambda _i: self._refresh(force=True))
        page.addWidget(self.tabs, 3)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.Shape.NoFrame)
        scroll.setWidget(root)
        self.setCentralWidget(scroll)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(33)                        # ~30 fps; plots redraw only on new frames

    # ------------------------------------------------------------ layout --
    def _build_header(self):
        left = QtWidgets.QVBoxLayout()
        left.setSpacing(0)
        left.addWidget(QtWidgets.QLabel("Dual-Stage Grid Power Quality System", objectName="title"))
        left.addWidget(QtWidgets.QLabel("Stage 1  APFC switched capacitor bank   ·   Stage 2  Active Harmonic "
                                        "Filter (TB6612FNG, 80 kHz unipolar)", objectName="subtitle"))
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(4)
        self.pill = Pill()
        self.pill.set("connecting...", MUTED)
        self.note = QtWidgets.QLabel("", objectName="subtitle")
        self.note.setAlignment(QtCore.Qt.AlignmentFlag.AlignRight)
        right.addWidget(self.pill, 0, QtCore.Qt.AlignmentFlag.AlignRight)
        right.addWidget(self.note)
        lay = QtWidgets.QHBoxLayout()
        lay.addLayout(left, 1)
        lay.addLayout(right)
        return lay

    def _build_common(self):
        ms = lambda x: "t = %.2f ms" % x
        self.c_v = Card("Grid voltage  V(t)", "")
        self.p_v = Plot("V", "time within one 50 Hz cycle  [ms]", 250)
        self.l_v = self.p_v.line(C_V, 2.2)
        self.p_v.hover([(self.l_v, "V", lambda y: "%.1f V" % y)], ms)
        self.c_v.body.addWidget(self.p_v)

        self.c_i = Card("Grid current  I(t)", "")
        self.p_i = Plot("A", "time within one 50 Hz cycle  [ms]", 250)
        self.l_i = self.p_i.line(C_I, 2.2)
        self.p_i.hover([(self.l_i, "I", lambda y: "%.3f A" % y)], ms)
        self.c_i.body.addWidget(self.p_i)
        for p in (self.p_v, self.p_i):
            p.setXRange(0, 1000.0 / F_GRID, padding=0.01)
        return _row(self.c_v, self.c_i)

    def _build_pfc(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(14)

        # gauge
        gcard = Card("Power factor", "target 0.95")
        self.gauge = PFGauge()
        gcard.body.addWidget(self.gauge, 1)
        leg = QtWidgets.QLabel('<span style="color:%s">▼</span> DPF (displacement) &nbsp;&nbsp; '
                               '<span style="color:%s">┆</span> target' % (C_DPF, FG), objectName="cardSub")
        leg.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
        gcard.body.addWidget(leg)

        # metrics
        self.s_p = Stat("Active power P")
        self.s_q = Stat("Reactive power Q₁")
        self.s_s = Stat("Apparent power S")
        self.s_d = Stat("Distortion power D")
        self.s_vr = Stat("Voltage")
        self.s_ir = Stat("Current")
        self.s_cap = Stat("Capacitance on")
        self.s_req = Stat("Capacitance required")
        stats = QtWidgets.QWidget()
        stats.setLayout(_grid([self.s_p, self.s_q, self.s_s, self.s_d,
                               self.s_vr, self.s_ir, self.s_cap, self.s_req], 4))
        stats.layout().setContentsMargins(0, 0, 0, 0)

        # capacitor bank
        bank = Card("Capacitor bank", "")
        self.bank_sub = bank.sub
        self.btn_auto = QtWidgets.QPushButton("AUTO MODE")
        self.btn_auto.setMinimumHeight(40)
        self.btn_auto.clicked.connect(self._on_auto)
        bank.body.addWidget(self.btn_auto)
        caps = QtWidgets.QHBoxLayout()
        caps.setSpacing(8)
        self.btn_cap = []
        for j in range(3):
            b = QtWidgets.QPushButton("%g µF" % CAP_UF[j])
            b.setMinimumHeight(58)
            b.clicked.connect(self._cap_cb(j))
            caps.addWidget(b)
            self.btn_cap.append(b)
        bank.body.addLayout(caps)
        self.apfc_txt = QtWidgets.QLabel("", objectName="cardSub")
        self.apfc_txt.setWordWrap(True)
        bank.body.addWidget(self.apfc_txt)
        bank.body.addStretch(1)

        lay.addLayout(_row(gcard, stats, bank, stretch=[3, 6, 3]))

        # trends
        sec = lambda x: "%.1f s" % x
        tc = Card("Power factor trend", "last %d s" % HIST_S)
        self.p_pf = Plot("PF", "seconds", 240)
        self.p_pf.legend()
        self.l_pf = self.p_pf.line(C_PF, 2.0, "PF")
        self.l_dpf = self.p_pf.line(C_DPF, 1.6, "DPF")
        self.p_pf.addItem(pg.InfiniteLine(0.95, angle=0, pen=pg.mkPen(C_BAD, width=1.2,
                                                                    style=QtCore.Qt.PenStyle.DashLine)))
        self.p_pf.hover([(self.l_pf, "PF", lambda y: "%.3f" % y), (self.l_dpf, "DPF", lambda y: "%.3f" % y)], sec)
        tc.body.addWidget(self.p_pf)

        cc = Card("Capacitance: switched vs required", "µF")
        self.p_c = Plot("µF", "seconds", 240)
        self.p_c.legend()
        self.l_cap = self.p_c.line(C_AFTER, 2.2, "switched in")
        self.l_need = self.p_c.line(C_BEFORE, 1.6, "required for 0.95", dash=True)
        self.p_c.hover([(self.l_cap, "on", lambda y: "%.1f µF" % y), (self.l_need, "req", lambda y: "%.2f µF" % y)], sec)
        cc.body.addWidget(self.p_c)
        for p in (self.p_pf, self.p_c):
            p.setXRange(-HIST_S, 0, padding=0.01)
        lay.addLayout(_row(tc, cc), 1)
        return w

    def _build_hm(self):
        w = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(14)

        self.s_thdi = Stat("THD-I  grid")
        self.s_thdc = Stat("THD-I  corrected")
        self.s_thdv = Stat("THD-V")
        self.s_crest = Stat("Crest factor (I)", "sine = 1.41")
        self.s_rho = Stat("AHF tracking ρ", "I_ahf vs target")
        self.s_match = Stat("Spectrum shape match")
        self.s_leak = Stat("H1 leak", "fundamental in I_ahf")
        self.s_i1 = Stat("Fundamental I₁")
        stats = QtWidgets.QWidget()
        stats.setLayout(_grid([self.s_thdi, self.s_thdc, self.s_thdv, self.s_crest,
                               self.s_rho, self.s_match, self.s_leak, self.s_i1], 4))
        stats.layout().setContentsMargins(0, 0, 0, 0)

        ahf = Card("AHF  ·  TB6612FNG H-bridge", "")
        self.ahf_sub = ahf.sub
        brow = QtWidgets.QHBoxLayout()
        brow.setSpacing(8)
        self.btn_act = QtWidgets.QPushButton("ACTIVE")
        self.btn_stb = QtWidgets.QPushButton("STANDBY")
        for b, en in ((self.btn_act, 1), (self.btn_stb, 0)):
            b.setMinimumHeight(40)
            b.clicked.connect(lambda _c=False, e=en: self._ahf_cmd(e))
            brow.addWidget(b)
        ahf.body.addLayout(brow)
        self.diag = QtWidgets.QLabel("", objectName="mono")
        ahf.body.addWidget(self.diag)
        ahf.body.addStretch(1)
        lay.addLayout(_row(stats, ahf, stretch=[9, 4]))

        ms = lambda x: "t = %.2f ms" % x
        ca = Card("AHF injected current vs target reference", "each normalised to its own peak")
        self.ahf_card_sub = ca.sub
        self.p_a = Plot("p.u.", "time within one 50 Hz cycle  [ms]", 250)
        self.p_a.legend()
        self.l_a = self.p_a.line(C_AHF, 2.2, "I_ahf measured (ACS712 #2)")
        self.l_t = self.p_a.line(C_REF, 1.6, "target  -(I - I₁)", dash=True)
        self.p_a.hover([(self.l_a, "I_ahf", lambda y: "%.2f pu" % y), (self.l_t, "target", lambda y: "%.2f pu" % y)], ms)
        self.p_a.setYRange(-1.2, 1.2, padding=0)
        ca.body.addWidget(self.p_a)

        cc = Card("Grid current: before vs after compensation", "")
        self.corr_sub = cc.sub
        self.p_c2 = Plot("A", "time within one 50 Hz cycle  [ms]", 250)
        self.p_c2.legend()
        self.l_c0 = self.p_c2.line(C_BEFORE, 1.4, "before")
        self.l_c = self.p_c2.line(C_AFTER, 2.4, "after  (I + I_ahf)")
        self.p_c2.hover([(self.l_c0, "before", lambda y: "%.3f A" % y), (self.l_c, "after", lambda y: "%.3f A" % y)], ms)
        cc.body.addWidget(self.p_c2)
        for p in (self.p_a, self.p_c2):
            p.setXRange(0, 1000.0 / F_GRID, padding=0.01)
        lay.addLayout(_row(ca, cc), 1)

        hx = np.arange(2, N_H + 1)
        sc = Card("Grid current spectrum", "h2–h15, % of I₁")
        self.spec_sub = sc.sub
        self.p_h = Plot("% of I₁", "harmonic order", 250)
        self.p_h.legend()
        self.b_before = pg.BarGraphItem(x=hx - 0.2, height=np.zeros(hx.size), width=0.38,
                                        brush=C_BEFORE, pen=pg.mkPen(None), name="before")
        self.b_after = pg.BarGraphItem(x=hx + 0.2, height=np.zeros(hx.size), width=0.38,
                                       brush=C_AFTER, pen=pg.mkPen(None), name="after")
        for b in (self.b_before, self.b_after):
            self.p_h.addItem(b)
        sc.body.addWidget(self.p_h)

        ac = Card("AHF current spectrum", "% of largest harmonic")
        self.ahfspec_sub = ac.sub
        hx1 = np.arange(1, N_H + 1)
        self.p_ha = Plot("%", "harmonic order", 250)
        self.p_ha.legend()
        self.b_ha = pg.BarGraphItem(x=hx1, height=np.zeros(N_H), width=0.62, brush=C_AHF,
                                    pen=pg.mkPen(None), name="I_ahf measured")
        self.p_ha.addItem(self.b_ha)
        self.p_ref = self.p_ha.plot(hx, np.zeros(hx.size), pen=None, symbol="d", symbolSize=10,
                                    symbolBrush=C_REF, symbolPen=pg.mkPen("w", width=1),
                                    name="required (grid harmonics)")
        self.p_ha.setYRange(0, 125, padding=0)
        ac.body.addWidget(self.p_ha)
        for p, x0 in ((self.p_h, 1.4), (self.p_ha, 0.4)):
            p.setXRange(x0, N_H + 0.6, padding=0)
            p.getPlotItem().getAxis("bottom").setTicks([[(k, str(k)) for k in range(1, N_H + 1)]])
            p.grid.x = False
        lay.addLayout(_row(sc, ac), 1)
        return w

    # ---------------------------------------------------------- commands --
    def _say(self, text):
        self._note, self._note_t = text, time.monotonic()

    def _send(self, cmd):
        ok = self.link.send(cmd)
        self._say(("sent  " if ok else "NOT SENT (no link)  ") + cmd.strip())
        return ok

    def _locks(self):
        with self.telem.lock:
            return int(self.telem.k["mask"]), int(self.telem.k["state"]), int(self.telem.pfc["state"])

    def _on_auto(self):
        mask, _lst, st = self._locks()
        if mask == 0:   # AUTO -> MANUAL: freeze every relay where it is now
            for j in range(3):
                self._send("$C,%d,1,%d\n" % (j, st >> j & 1))
            self.telem.optimistic(k={"mask": 7, "state": st})
        else:           # MANUAL/partial -> AUTO: release all locks
            for j in range(3):
                self._send("$C,%d,0,0\n" % j)
            self.telem.optimistic(k={"mask": 0, "state": 0})
        self._refresh(force=True)

    def _cap_cb(self, j):
        def cb():
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
            self._refresh(force=True)
        return cb

    def _ahf_cmd(self, en):
        self._send("$A,%d\n" % en)
        self.telem.optimistic(ahf={"active": en})
        self._refresh(force=True)

    # ------------------------------------------------------------ update --
    def _yscale(self, plot, key, arrays, floor):
        m = max((float(np.max(np.abs(a))) for a in arrays if a is not None and a.size), default=0.0)
        tgt = max(m * 1.18, floor)
        cur = self._lim.get(key, tgt)
        cur = tgt if tgt > cur else 0.9 * cur + 0.1 * tgt
        self._lim[key] = cur
        plot.setYRange(-cur, cur, padding=0)

    def _tick(self):
        self.gauge.step()
        self._refresh()

    def _refresh(self, force=False):
        s = self.telem.snapshot()
        now = time.monotonic()
        if force or now - self._last_status > 0.25:
            self._last_status = now
            live = s["age"] < 2.5
            self.pill.set(self.link.status if live else "NO DATA  ·  " + self.link.status,
                          C_GOOD if live else C_BAD)
            note = self._note if now - self._note_t < 4 else ""
            self.note.setText("frames %d  ·  %.1f fps  ·  bad lines %d    %s"
                              % (s["frames"], s["fps"], s["bad"], note))
        if not s["frames"] or (s["frames"] == self._frames_seen and not force):
            return
        new_frame = s["frames"] != self._frames_seen
        self._frames_seen = s["frames"]
        w, m, pfc, ahf, k, r = s["wave"], s["m"], s["pfc"], s["ahf"], s["k"], s["r"]

        f_hz = m["freq"] if 40.0 < m["freq"] < 70.0 else F_GRID   # old firmware sends no freq

        def xs(y):
            return np.arange(y.size) * (1000.0 / f_hz / y.size)

        yv, yi, ya, yt, yc = (w.get(n) for n in ("V", "I", "A", "T", "C"))
        hi = s["h"]["I"]
        hc = _spectrum(yc)
        thd_after = _thd(hc)
        if new_frame:
            self.hist.append((now - self._t0, m["pf"], m["dpf"], pfc["cap"], pfc["c_need"],
                              m["thd_i"], thd_after))

        # ---- common: V and I ------------------------------------------------
        if yv is not None and yv.size:
            self.l_v.setData(xs(yv), yv)
            self._yscale(self.p_v, "v", [yv], 10.0)
        if yi is not None and yi.size:
            self.l_i.setData(xs(yi), yi)
            self._yscale(self.p_i, "i", [yi], 0.05)
        self.c_v.sub.setText("%.1f V rms  ·  %.2f Hz  ·  THD %.1f %%" % (m["vrms"], f_hz, m["thd_v"]))
        self.c_i.sub.setText("%.3f A rms  ·  THD %.1f %%  ·  crest %.2f" % (m["irms"], m["thd_i"], m["crest"]))

        if self.tabs.currentIndex() == 0:
            self._update_pfc(m, pfc, k)
        else:
            self._update_hm(m, ahf, r, k, hi, s["h"]["A"], hc, thd_after, ya, yt, yi, yc, xs)

    def _update_pfc(self, m, pfc, k):
        kind = "lagging" if m["q"] > 0 else "leading" if m["q"] < 0 else ""
        self.gauge.set(m["pf"], m["dpf"], "%s   ·   DPF %.3f" % (kind, m["dpf"]))
        self.s_p.set("%.1f" % m["p"], "W")
        self.s_q.set("%+.1f" % m["q"], "var", kind)
        self.s_s.set("%.1f" % m["s"], "VA")
        self.s_d.set("%.1f" % m["dva"], "VA", "from harmonics")
        self.s_vr.set("%.1f" % m["vrms"], "V rms", "peak %.1f V" % m["vpk"])
        self.s_ir.set("%.3f" % m["irms"], "A rms", "peak %.3f A" % m["ipk"])
        self.s_cap.set("%.1f" % pfc["cap"], "µF")
        self.s_req.set("%.2f" % pfc["c_need"], "µF", "for PF 0.95")

        mask = int(k["mask"])
        st, tgt = int(pfc["state"]), int(pfc["target"])
        if mask == 0:
            self.btn_auto.setText("AUTO MODE  ·  ON")
            _css(self.btn_auto, _btn_style("#e7f6ee", "#9fd8bd", C_GOOD))
        else:
            self.btn_auto.setText("AUTO MODE  ·  %s" % ("OFF (MANUAL)" if mask == 7 else "PARTIAL"))
            _css(self.btn_auto, _btn_style("#fdf4e2", "#e7c27a", C_WARN, 2))
        for j, b in enumerate(self.btn_cap):
            on, locked, want = st >> j & 1, mask >> j & 1, tgt >> j & 1
            state = "ON" if on else "OFF"
            if want != on:
                state += " → ON (guard)" if want else " → OFF"
            b.setText("%g µF\n%s · %s" % (CAP_UF[j], "MAN" if locked else "AUTO", state))
            face, edge = ("#e7f6ee", "#9fd8bd") if on else (CARD, "#d6d9de")
            if locked:
                edge = C_WARN
            _css(b, _btn_style(face, edge, C_GOOD if on else FG2, 2 if locked else 1))
        self.bank_sub.setText("%d of 3 on" % bin(st).count("1"))
        self.apfc_txt.setText("Controller: <b>%s</b>  ·  installed %.1f µF  ·  required %.2f µF<br>"
                              "Click a step to override it; click again to cycle MAN ON → MAN OFF → AUTO.<br>"
                              "A capacitor that was just switched off waits %g s before it can close again."
                              % ("HOLD" if int(pfc["hold"]) else "SWITCHING", pfc["cap"], pfc["c_need"], CAP_RECLOSE_S))

        if self.hist:
            h = np.asarray(self.hist)
            t = h[:, 0] - h[-1, 0]
            keep = t >= -HIST_S
            t, h = t[keep], h[keep]
            self.l_pf.setData(t, np.abs(h[:, 1]))
            self.l_dpf.setData(t, np.abs(h[:, 2]))
            lo = min(0.5, float(np.min(np.abs(h[:, 1:3]))) - 0.05)
            self.p_pf.setYRange(max(lo, 0.0), 1.02, padding=0)
            self.l_cap.setData(t, h[:, 3])
            self.l_need.setData(t, h[:, 4])
            self.p_c.setYRange(0, max(sum(CAP_UF), float(np.max(h[:, 3:5]))) * 1.08, padding=0)

    def _update_hm(self, m, ahf, r, k, hi, ha, hc, thd_after, ya, yt, yi, yc, xs):
        active = int(ahf["active"])
        rho = _corr(ya, yt)
        thd_col = lambda v: C_GOOD if v <= 5 else C_WARN if v <= 8 else C_BAD
        self.s_thdi.set("%.1f" % m["thd_i"], "%", "before compensation", thd_col(m["thd_i"]))
        if math.isnan(thd_after):
            self.s_thdc.set("--", "", "no corrected waveform")
        else:
            red = (1 - thd_after / m["thd_i"]) * 100 if m["thd_i"] > 1e-6 else 0.0
            self.s_thdc.set("%.1f" % thd_after, "%", "%.0f %% lower than grid" % red, thd_col(thd_after))
        self.s_thdv.set("%.1f" % m["thd_v"], "%", "IEEE 519: ≤ 8 %", thd_col(m["thd_v"]))
        self.s_crest.set("%.2f" % m["crest"])
        self.s_rho.set("--" if math.isnan(rho) else "%.3f" % rho, "",
                       "ACTIVE" if active else "STANDBY",
                       FG if math.isnan(rho) else C_GOOD if rho > 0.9 else C_WARN if rho > 0.6 else C_BAD)
        self.s_i1.set("%.3f" % hi[0], "A rms")

        _css(self.btn_act, _btn_style("#e3f6ef", C_AHF, C_GOOD, 2) if active else _btn_style(CARD, "#d6d9de", FG2))
        _css(self.btn_stb, _btn_style("#fdf4e2", C_WARN, C_WARN, 2) if not active else _btn_style(CARD, "#d6d9de", FG2))
        self.ahf_sub.setText("ACTIVE  ·  loop residual %.1f %%  ·  duty pk %.0f %%"
                             % (ahf["resid"], 100 * ahf["duty"]) if active else "STANDBY")

        # AHF vs target (normalised)
        def norm(y):
            if y is None or not y.size:
                return None
            pk = float(np.max(np.abs(y)))
            return y / pk if pk > 1e-9 else y
        an, tn = norm(ya), norm(yt)
        if an is not None:
            self.l_a.setData(xs(ya), an)
        if tn is not None:
            self.l_t.setData(xs(yt), tn)
        self.ahf_card_sub.setText("normalised to own peak  ·  peak %.3f / %.3f A  ·  ρ %s"
                                  % (ahf["peak"], ahf["ref_peak"], "--" if math.isnan(rho) else "%.3f" % rho))

        # before / after
        if yi is not None and yi.size:
            self.l_c0.setData(xs(yi), yi)
        if yc is not None and yc.size:
            self.l_c.setData(xs(yc), yc)
        self._yscale(self.p_c2, "c", [yi, yc], 0.05)
        self.corr_sub.setText("THD %.1f %% → %s" % (m["thd_i"], "--" if math.isnan(thd_after) else "%.1f %%" % thd_after))

        # spectra
        pct = lambda h: 100.0 * h[1:] / h[0] if h[0] > 1e-6 else np.zeros(N_H - 1)
        before, after = pct(hi), pct(hc)
        self.b_before.setOpts(height=before)
        self.b_after.setOpts(height=after)
        top = max(5.0, float(np.max(before)) * 1.15)
        cur = self._lim.get("h", top)
        cur = top if top > cur else 0.9 * cur + 0.1 * top
        self._lim["h"] = cur
        self.p_h.setYRange(0, cur, padding=0)
        self.spec_sub.setText("%% of I₁  ·  I₁ = %.3f A rms" % hi[0])

        ref_max, ahf_max = float(np.max(hi[1:])), float(np.max(ha[1:]))
        ref_n = 100.0 * hi[1:] / ref_max if ref_max > 1e-6 else np.zeros(N_H - 1)
        ahf_n = 100.0 * ha / ahf_max if ahf_max > 1e-6 else np.zeros(N_H)
        self.b_ha.setOpts(height=np.minimum(ahf_n, 122.0))
        self.p_ref.setData(np.arange(2, N_H + 1), ref_n)
        den = np.linalg.norm(ahf_n[1:]) * np.linalg.norm(ref_n)
        match = 100.0 * float(np.dot(ahf_n[1:], ref_n)) / den if den > 1e-9 else float("nan")
        self.s_match.set("--" if math.isnan(match) else "%.0f" % match, "%", "I_ahf vs required",
                         FG if math.isnan(match) else C_GOOD if match > 90 else C_WARN if match > 70 else C_BAD)
        self.s_leak.set("%.0f" % ahf_n[0] if ahf_max > 1e-6 else "--", "%" if ahf_max > 1e-6 else "")

        clip = int(r["clip_v"] + r["clip_i"] + r["clip_a"])
        self.diag.setText(
            "ADC     offset    min    max  clip\n"
            "  V   %7.1f  %5d  %5d %5d\n"
            "  I   %7.1f  %5d  %5d %5d\n"
            "  A   %7.1f  %5d  %5d %5d\n"
            "ovr %d  drop %d  tx-skip %d  %s\n"
            "UART  cmd ok %d  bad %d  rx %d B\n"
            "AHF   duty pk %.2f  residual %.1f %%"
            % (r["off_v"], r["min_v"], r["max_v"], r["clip_v"],
               r["off_i"], r["min_i"], r["max_i"], r["clip_i"],
               r["off_a"], r["min_a"], r["max_a"], r["clip_a"],
               r["adc_ovr"], r["win_drop"], r["tx_skip"], "CLIPPING!" if clip else "",
               k["ok"], k["bad"], k["rx"], ahf["duty"], ahf["resid"]))
        _css(self.diag, "color:%s" % (C_BAD if clip else FG2))


# =========================================================================== #
def main():
    ap = argparse.ArgumentParser(description="APFC + AHF real-time dashboard")
    ap.add_argument("--port", help="serial port, e.g. COM35 or /dev/ttyACM0 (default: auto)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--demo", action="store_true", help="run against the built-in simulator")
    args = ap.parse_args()

    if not args.demo and serial is None:
        sys.exit("pyserial is not installed:  py -m pip install pyserial   (or run with --demo)")

    pg.setConfigOptions(antialias=True, background=CARD, foreground=FG2)
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    telem = Telemetry()
    link = DemoLink(telem) if args.demo else SerialLink(args.port, args.baud, telem)
    link.start()
    win = Dashboard(telem, link)
    win.show()
    try:
        app.exec()
    finally:
        link.close()


if __name__ == "__main__":
    main()
