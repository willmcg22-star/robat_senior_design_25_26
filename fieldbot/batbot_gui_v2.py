"""
BatBot Mission Control GUI
==========================
Dependencies:
    pip install PyQt5 pyqtgraph numpy scipy pyserial pyubx2 gpxpy pygnssutils

Serial devices expected (can be overridden via argparse or the GUI):
    /dev/sonar   - chirp MCU (115200 baud)
    /dev/tendons - tendon controller (auto-detected or pass port)
    /dev/gps     - ZED-F9P GPS (115200 baud)

Run:
    python batbot_gui.py                        # no hardware, no sim
    python batbot_gui.py --sim                  # fully simulated data, no serial needed
    python batbot_gui.py --sim --sim-target 1.2 # sim with a target at 1.2 m range
    python batbot_gui.py --sonar /dev/ttyUSB0 --gps /dev/ttyUSB1 --tendons /dev/ttyUSB2

Simulation mode (--sim):
    * SonarSimWorker  – generates synthetic ADC frames containing a realistic
                        downward chirp (100→30 kHz) plus additive noise.
                        Use --sim-target <metres> to inject a time-delayed echo
                        at a chosen one-way range (e.g. 0.5 m → ~2.9 ms delay).
                        Left and right channels get independent noise so the
                        spectrograms look distinct.
    * GPSSimWorker    – walks a small elliptical path around a fixed origin
                        (Virginia Tech by default) and synthesises RTK-quality
                        lat/lon/alt at 2 Hz, populating the ENU map track.
    * TendonSimWorker – slowly sweeps all 12 motors through sinusoidal
                        trajectories so the tendon panel shows live motion.
"""

import sys
import os
import argparse
import time
import struct
import threading
from datetime import datetime
from queue import Queue, Empty

import numpy as np
from scipy import signal
from scipy.io import savemat

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QLabel, QPushButton, QLineEdit, QTabWidget,
    QGroupBox, QProgressBar, QStatusBar, QSpinBox, QDoubleSpinBox,
    QComboBox, QSplitter, QFrame, QFileDialog, QMessageBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QFont, QPalette, QColor

import pyqtgraph as pg
from pyqtgraph import PlotWidget, ImageItem, GraphicsLayoutWidget

# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────
SOUND_SPEED_AIR = 343.0          # m/s at ~20°C
FS = 1_000_000                   # ADC sample rate (1 MHz)
N = 16_000                       # samples per record
T_CHIRP = 3e-3                   # chirp duration (s)
F0_CHIRP = 100_000               # chirp start freq (Hz)
F1_CHIRP = 30_000                # chirp end freq (Hz)
NFFT = 512
NOVERLAP = 400
N_CHIRP = int(FS * T_CHIRP)
N_RECORD = N - N_CHIRP
OFFS_CHIRP = 2048
GAIN_CHIRP = 512
NUM_TENDONS = 12

# Serial opcodes (matches run_chirp.py)
OP_AMP_START  = 0xfe
OP_AMP_STOP   = 0xff
OP_START_JOB  = 0x10
OP_GET_CHIRP  = 0x2f
OP_CHIRP_EN   = 0x2e
DO_CHIRP      = 0x01
DONT_CHIRP    = 0x00

# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────
def build_chirp_bytes():
    """Build the biased chirp byte array to send to the MCU."""
    Ts = 1 / FS
    tv = np.arange(0, T_CHIRP - Ts / 2, Ts)
    chirp = signal.chirp(tv, F0_CHIRP, T_CHIRP, F1_CHIRP, method='linear')
    win = signal.windows.hann(N_CHIRP, False)
    biased = (np.rint(OFFS_CHIRP + GAIN_CHIRP * chirp)).astype(int)
    buf = bytearray()
    for num in biased.tolist():
        b = num.to_bytes(2, byteorder='big')
        buf.append(b[1])
        buf.append(b[0])
    return buf

def bin2dec(raw: bytes) -> np.ndarray:
    """Convert raw 2-byte big-endian ADC bytes to float array."""
    arr = np.frombuffer(raw, dtype=np.dtype('>u2')).astype(np.float64)
    return arr

def compute_spectrogram(raw: bytes, time_offset: int = 4000):
    """Return (S_dB, freqs_kHz, times_ms) for one channel."""
    data = bin2dec(raw)
    data -= np.mean(data)
    data = data[time_offset:]
    win = signal.windows.hann(NFFT)
    f, t, S = signal.spectrogram(
        data, fs=FS, window=win, nperseg=NFFT,
        noverlap=NOVERLAP, scaling='spectrum'
    )
    S_dB = 10 * np.log10(S + 1e-12)
    S_dB -= np.max(S_dB)
    # Convert axes
    f_kHz = f / 1e3
    t_ms  = t * 1e3
    return S_dB, f_kHz, t_ms

def lat_lon_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """Flat-earth ENU from reference point (degrees, metres)."""
    R = 6_378_137.0
    dN = (lat - lat0) * np.pi / 180 * R
    dE = (lon - lon0) * np.pi / 180 * R * np.cos(lat0 * np.pi / 180)
    dU = alt - alt0
    return dE, dN, dU

# ─────────────────────────────────────────────────────────────────────────────
# Worker threads
# ─────────────────────────────────────────────────────────────────────────────
class Signals(QObject):
    new_frame    = pyqtSignal(bytes, bytes)   # raw1, raw2
    gps_update   = pyqtSignal(dict)
    tendon_angle = pyqtSignal(int, float)     # motor_idx, angle
    error        = pyqtSignal(str)

class SonarWorker(QThread):
    """Continuously fires chirps and emits raw ADC frames."""
    def __init__(self, port: str, signals: Signals):
        super().__init__()
        self.port = port
        self.signals = signals
        self._stop = threading.Event()
        self.chirp_bytes = build_chirp_bytes()

    def run(self):
        try:
            import serial
            ser = serial.Serial(self.port, 115200, timeout=5)
            # Init amp and send chirp data once
            ser.write([OP_AMP_START])
            ser.write([OP_GET_CHIRP])
            ser.write(self.chirp_bytes)
            # Flush ADCs
            ser.write([OP_START_JOB, DONT_CHIRP])
            ser.read(2 * N); ser.read(2 * N)

            while not self._stop.is_set():
                ser.write([OP_START_JOB, DO_CHIRP])
                raw1 = ser.read(2 * N)
                raw2 = ser.read(2 * N)
                if len(raw1) == 2 * N and len(raw2) == 2 * N:
                    self.signals.new_frame.emit(raw1, raw2)

            ser.write([OP_AMP_STOP])
            ser.close()
        except Exception as e:
            self.signals.error.emit(f"Sonar: {e}")

    def stop(self):
        self._stop.set()


class GPSWorker(QThread):
    """Reads ZED-F9P UBX NAV-PVT messages and emits GPS dicts."""
    def __init__(self, port: str, signals: Signals,
                 ntrip_user=None, mountpoint="", ntrip_server="rtk2go.com",
                 ntrip_port=2101, ntrip_password="none"):
        super().__init__()
        self.port = port
        self.signals = signals
        self.ntrip_user = ntrip_user
        self.mountpoint = mountpoint
        self.ntrip_server = ntrip_server
        self.ntrip_port = ntrip_port
        self.ntrip_password = ntrip_password
        self._stop = threading.Event()

    def run(self):
        try:
            from serial import Serial
            from pyubx2 import UBXReader
            ser = Serial(self.port, 115200, timeout=3)
            ubr = UBXReader(ser)

            # Optionally start NTRIP
            ntrip_q = Queue()
            if self.ntrip_user:
                try:
                    from pygnssutils import GNSSNTRIPClient, VERBOSITY_LOW
                    client = GNSSNTRIPClient(verbosity=VERBOSITY_LOW)
                    client.run(
                        server=self.ntrip_server, port=self.ntrip_port,
                        mountpoint=self.mountpoint, ntripuser=self.ntrip_user,
                        ntrippassword=self.ntrip_password, output=ntrip_q
                    )
                except Exception as e:
                    self.signals.error.emit(f"NTRIP: {e}")

            while not self._stop.is_set():
                # Forward NTRIP corrections
                try:
                    raw, _ = ntrip_q.get_nowait()
                    ser.write(raw)
                except Empty:
                    pass

                try:
                    _, msg = ubr.read()
                except Exception:
                    continue

                if msg and hasattr(msg, 'lat'):
                    self.signals.gps_update.emit({
                        'lat':  msg.lat,
                        'lon':  msg.lon,
                        'alt':  getattr(msg, 'hMSL', 0) / 1000.0,
                        'pdop': getattr(msg, 'pDOP', 0) / 100.0,
                        'fix':  getattr(msg, 'fixType', 0),
                        'ts':   datetime.utcnow().isoformat(),
                    })
            ser.close()
        except Exception as e:
            self.signals.error.emit(f"GPS: {e}")

    def stop(self):
        self._stop.set()


# ─────────────────────────────────────────────────────────────────────────────
# Simulation workers  (activated by --sim flag, no serial hardware needed)
# ─────────────────────────────────────────────────────────────────────────────

class SonarSimWorker(QThread):
    """
    Generates synthetic ADC frames that look like real bat chirp returns.

    Signal model per frame
    ──────────────────────
    1. Transmit chirp  (100 → 30 kHz linear, Hann-windowed, N_CHIRP samples)
    2. Dead-time       (silence while amp settles, ~time_offset samples)
    3. Optional echo   (attenuated, time-delayed copy of the chirp at
                        range = sim_target_m; one-way delay =
                        sim_target_m / SOUND_SPEED_AIR seconds)
    4. Additive white Gaussian noise throughout (σ ≈ 60 ADC counts)

    Both channels are identical in structure but use independent noise
    seeds so they look different on the spectrograms.
    """
    def __init__(self, signals: Signals, sim_target_m: float = 0.8,
                 frame_rate_hz: float = 5.0):
        super().__init__()
        self.signals        = signals
        self.sim_target_m   = sim_target_m
        self.frame_rate_hz  = frame_rate_hz
        self._stop          = threading.Event()
        self._rng           = np.random.default_rng(42)

        # Pre-build transmit chirp
        Ts   = 1.0 / FS
        tv   = np.arange(0, T_CHIRP - Ts / 2, Ts)
        raw  = signal.chirp(tv, F0_CHIRP, T_CHIRP, F1_CHIRP, method='linear')
        win  = signal.windows.hann(len(tv), False)
        self._tx = (raw * win).astype(np.float64)   # normalised -1 … +1

        # Echo delay in samples
        one_way_s          = sim_target_m / SOUND_SPEED_AIR
        self._echo_delay   = int(one_way_s * FS)    # samples after chirp end

    def _make_frame(self, noise_scale: float = 60.0,
                    echo_amplitude: float = 180.0) -> bytes:
        """Return one frame of 2*N bytes (big-endian uint16, ADC counts)."""
        frame = np.zeros(N, dtype=np.float64)

        # 1. Transmit chirp at the start
        frame[:N_CHIRP] = OFFS_CHIRP + GAIN_CHIRP * self._tx

        # 2. Echo – starts at N_CHIRP + echo_delay (after amp dead-time)
        echo_start = N_CHIRP + self._echo_delay
        echo_end   = echo_start + len(self._tx)
        if echo_end < N:
            frame[echo_start:echo_end] += echo_amplitude * self._tx

        # 3. Noise
        frame += self._rng.normal(0, noise_scale, N)

        # 4. Bias so we stay in ADC range (0 … 4095), clip & pack
        frame += OFFS_CHIRP
        frame  = np.clip(frame, 0, 4095).astype(np.uint16)

        # Pack as big-endian uint16 (matches bin2dec expectation)
        buf = bytearray(2 * N)
        for i, v in enumerate(frame):
            buf[2*i]   = (v >> 8) & 0xFF
            buf[2*i+1] =  v       & 0xFF
        return bytes(buf)

    def run(self):
        interval = 1.0 / self.frame_rate_hz
        while not self._stop.is_set():
            t0   = time.time()
            raw1 = self._make_frame(noise_scale=55, echo_amplitude=200)
            raw2 = self._make_frame(noise_scale=70, echo_amplitude=160)
            self.signals.new_frame.emit(raw1, raw2)
            elapsed = time.time() - t0
            sleep_s = max(0.0, interval - elapsed)
            self._stop.wait(timeout=sleep_s)

    def stop(self):
        self._stop.set()


class GPSSimWorker(QThread):
    """
    Walks an elliptical path around a fixed origin at ~2 Hz.

    The orbit is deliberately tiny (±8 m East, ±5 m North) so the ENU
    map is immediately visible.  Altitude oscillates ±0.5 m.  Fix type
    is reported as 3 (3-D fix) with low pDOP to mimic RTK quality.
    """
    ORIGIN_LAT =  37.2285   # Virginia Tech area – change to your site
    ORIGIN_LON = -80.4234
    ORIGIN_ALT =  634.0     # metres MSL

    def __init__(self, signals: Signals, rate_hz: float = 2.0):
        super().__init__()
        self.signals  = signals
        self.rate_hz  = rate_hz
        self._stop    = threading.Event()
        self._rng     = np.random.default_rng(7)

    def run(self):
        interval = 1.0 / self.rate_hz
        R        = 6_378_137.0
        t        = 0.0
        period   = 30.0   # seconds for one full orbit

        lat0 = self.ORIGIN_LAT
        lon0 = self.ORIGIN_LON
        alt0 = self.ORIGIN_ALT

        # Convert desired ENU radii to degrees
        r_north_m =  5.0
        r_east_m  =  8.0
        dlat_per_m = 1.0 / (np.pi / 180 * R)
        dlon_per_m = 1.0 / (np.pi / 180 * R * np.cos(lat0 * np.pi / 180))

        while not self._stop.is_set():
            t0    = time.time()
            angle = 2 * np.pi * (t / period)

            lat = lat0 + r_north_m * np.sin(angle)       * dlat_per_m
            lon = lon0 + r_east_m  * np.cos(angle)       * dlon_per_m
            alt = alt0 + 0.5       * np.sin(angle * 2.3)

            # Add tiny RTK-quality noise (~5 mm)
            lat += self._rng.normal(0, 5e-3) * dlat_per_m
            lon += self._rng.normal(0, 5e-3) * dlon_per_m

            pdop = 1.1 + 0.05 * abs(np.sin(angle))

            self.signals.gps_update.emit({
                'lat':  float(lat),
                'lon':  float(lon),
                'alt':  float(alt),
                'pdop': float(pdop),
                'fix':  3,              # 3-D fix
                'ntrip': 'SIM',
                'ts':   datetime.utcnow().isoformat(),
            })

            t      += interval
            elapsed = time.time() - t0
            self._stop.wait(timeout=max(0.0, interval - elapsed))

    def stop(self):
        self._stop.set()


class TendonSimWorker(QThread):
    """
    Slowly sweeps 12 motors through independent sinusoidal trajectories
    so the tendon panel shows realistic live motion.

    Each motor i gets:
        angle_i(t) = max_i/2 * (1 + sin(2π·t/T_i + φ_i))
    where T_i and φ_i are randomised per motor so they don't all move
    in lockstep.
    """
    def __init__(self, signals: Signals, rate_hz: float = 10.0):
        super().__init__()
        self.signals  = signals
        self.rate_hz  = rate_hz
        self._stop    = threading.Event()

        rng = np.random.default_rng(99)
        # period 8–20 s, phase 0–2π, max_angle 60–150°
        self._periods    = rng.uniform(8,  20,  NUM_TENDONS)
        self._phases     = rng.uniform(0,  2*np.pi, NUM_TENDONS)
        self._max_angles = rng.uniform(60, 150, NUM_TENDONS)

    def run(self):
        interval = 1.0 / self.rate_hz
        t        = 0.0
        while not self._stop.is_set():
            t0 = time.time()
            for i in range(NUM_TENDONS):
                angle = self._max_angles[i] / 2.0 * (
                    1 + np.sin(2 * np.pi * t / self._periods[i] + self._phases[i])
                )
                self.signals.tendon_angle.emit(i, float(angle))
            t      += interval
            elapsed = time.time() - t0
            self._stop.wait(timeout=max(0.0, interval - elapsed))

    def stop(self):
        self._stop.set()


# ─────────────────────────────────────────────────────────────────────────────
# Spectrogram widget with dual axes (Time ms + Distance mm)
# ─────────────────────────────────────────────────────────────────────────────
class SpectrogramWidget(QWidget):
    """
    Displays a scrolling spectrogram with:
      - Y axis: Frequency (kHz)
      - X bottom: Time (ms)
      - X top: Distance (mm) via sound speed conversion
    """
    def __init__(self, title="Spectrogram", parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignCenter)
        font = self.title_label.font()
        font.setPointSize(9)
        font.setBold(True)
        self.title_label.setFont(font)
        layout.addWidget(self.title_label)

        self.glw = GraphicsLayoutWidget()
        layout.addWidget(self.glw)

        # Main spectrogram plot
        self.plot = self.glw.addPlot(row=0, col=0)
        self.plot.setLabel('left',   'Frequency', units='kHz')
        self.plot.setLabel('bottom', 'Time',      units='ms')
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setMenuEnabled(False)

        # Image item for spectrogram data
        self.img = ImageItem()
        self.plot.addItem(self.img)

        # Colour map (jet-like via pyqtgraph)
        cm = pg.colormap.get('CET-L16', source='colorcet') \
             if 'colorcet' in pg.colormap.listMaps() \
             else pg.colormap.get('plasma')
        self.img.setColorMap(cm)
        self.img.setLevels([-40, 0])

        # Colour bar
        self.cbar = pg.ColorBarItem(
            values=(-40, 0), colorMap=cm, label='dB', interactive=False
        )
        self.cbar.setImageItem(self.img, insert_in=self.plot)

        # Top axis: distance in mm
        self.dist_axis = pg.AxisItem('top')
        self.dist_axis.setLabel('Distance', units='mm')
        self.plot.layout.addItem(self.dist_axis, 1, 1)

        # State
        self._S    = None
        self._f    = None
        self._t    = None
        self.peak_dB = None

    def update_data(self, S_dB, f_kHz, t_ms):
        """Push a new full spectrogram frame."""
        self._S = S_dB
        self._f = f_kHz
        self._t = t_ms
        self.peak_dB = float(np.max(S_dB))
        self._redraw()

    def _redraw(self):
        if self._S is None:
            return
        S, f, t = self._S, self._f, self._t
        # Frequency band 25–110 kHz
        mask = (f >= 25) & (f <= 110)
        S_cut = S[mask, :]
        f_cut = f[mask]

        # Set image and scale so axes match data
        self.img.setImage(S_cut.T, autoLevels=False)

        t_range = float(t[-1] - t[0]) if len(t) > 1 else 1.0
        f_range = float(f_cut[-1] - f_cut[0]) if len(f_cut) > 1 else 1.0

        self.img.resetTransform()
        self.img.scale(t_range / max(S_cut.shape[1] - 1, 1),
                       f_range / max(S_cut.shape[0] - 1, 1))
        self.img.setPos(float(t[0]), float(f_cut[0]))

        self.plot.setXRange(float(t[0]), float(t[-1]), padding=0)
        self.plot.setYRange(float(f_cut[0]), float(f_cut[-1]), padding=0)

        # Update distance axis ticks (distance = speed × time)
        # t_ms → t_s → d_m → d_mm (one-way: divide by 2 for round-trip)
        t_ticks_ms = np.linspace(float(t[0]), float(t[-1]), 6)
        d_mm = (t_ticks_ms * 1e-3) * SOUND_SPEED_AIR * 1e3 / 2.0
        ticks = [(tv, f"{dv:.0f}") for tv, dv in zip(t_ticks_ms, d_mm)]
        self.dist_axis.setTicks([ticks])
        self.dist_axis.setRange(float(t[0]), float(t[-1]))


# ─────────────────────────────────────────────────────────────────────────────
# ENU map widget
# ─────────────────────────────────────────────────────────────────────────────
class ENUWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.plot = PlotWidget()
        self.plot.setLabel('bottom', 'East',  units='m')
        self.plot.setLabel('left',   'North', units='m')
        self.plot.setAspectLocked(True)
        self.plot.showGrid(x=True, y=True, alpha=0.3)
        self.plot.setMenuEnabled(False)
        layout.addWidget(self.plot)

        self.track_curve = self.plot.plot(pen=pg.mkPen('#378ADD', width=2))
        self.cur_scatter = pg.ScatterPlotItem(
            size=10, pen=pg.mkPen(None), brush=pg.mkBrush('#E24B4A')
        )
        self.plot.addItem(self.cur_scatter)

        self.east_pts  = []
        self.north_pts = []
        self.origin    = None

    def add_point(self, lat, lon, alt):
        if self.origin is None:
            self.origin = (lat, lon, alt)
        dE, dN, _ = lat_lon_to_enu(lat, lon, alt, *self.origin)
        self.east_pts.append(dE)
        self.north_pts.append(dN)
        if len(self.east_pts) > 2000:
            self.east_pts.pop(0)
            self.north_pts.pop(0)
        self.track_curve.setData(self.east_pts, self.north_pts)
        self.cur_scatter.setData([dE], [dN])
        return dE, dN

    def reset(self):
        self.origin = None
        self.east_pts.clear()
        self.north_pts.clear()
        self.track_curve.setData([], [])
        self.cur_scatter.setData([], [])


# ─────────────────────────────────────────────────────────────────────────────
# Tendon panel
# ─────────────────────────────────────────────────────────────────────────────
class TendonPanel(QWidget):
    def __init__(self, num_tendons=NUM_TENDONS, parent=None):
        super().__init__(parent)
        self.num = num_tendons
        layout = QVBoxLayout(self)

        grid = QGridLayout()
        grid.setSpacing(6)
        self.angle_labels  = []
        self.bars          = []
        self.status_labels = []

        for i in range(num_tendons):
            row, col = divmod(i, 4)
            box = QGroupBox(f"Motor {i+1}")
            box.setFixedHeight(90)
            bl = QVBoxLayout(box)
            bl.setContentsMargins(4, 4, 4, 4)
            bl.setSpacing(2)

            angle_lbl = QLabel("0.0°")
            angle_lbl.setFont(QFont("Courier New", 11, QFont.Bold))
            angle_lbl.setAlignment(Qt.AlignCenter)
            bl.addWidget(angle_lbl)

            bar = QProgressBar()
            bar.setRange(0, 180)
            bar.setValue(0)
            bar.setTextVisible(False)
            bar.setFixedHeight(6)
            bl.addWidget(bar)

            status = QLabel("IDLE")
            status.setAlignment(Qt.AlignCenter)
            status.setStyleSheet("font-size: 9px; color: gray;")
            bl.addWidget(status)

            grid.addWidget(box, row, col)
            self.angle_labels.append(angle_lbl)
            self.bars.append(bar)
            self.status_labels.append(status)

        layout.addLayout(grid)

        btn_row = QHBoxLayout()
        zero_btn = QPushButton("Zero all")
        zero_btn.clicked.connect(self.zero_all)
        btn_row.addWidget(zero_btn)

        calib_btn = QPushButton("Run calibration...")
        calib_btn.clicked.connect(self.run_calibration)
        btn_row.addWidget(calib_btn)
        layout.addLayout(btn_row)

    def set_angle(self, idx: int, angle: float, max_angle: float = 180.0):
        if 0 <= idx < self.num:
            self.angle_labels[idx].setText(f"{angle:.1f}°")
            self.bars[idx].setValue(int(np.clip(angle, 0, max_angle)))
            if angle == 0:
                color, text = "#1D9E75", "ZERO"
            elif angle > 150:
                color, text = "#BA7517", "LIMIT"
            else:
                color, text = "#378ADD", "OK"
            self.status_labels[idx].setText(text)
            self.status_labels[idx].setStyleSheet(f"font-size:9px;color:{color};")

    def zero_all(self):
        for i in range(self.num):
            self.set_angle(i, 0.0)

    def run_calibration(self):
        QMessageBox.information(
            self, "Calibration",
            "Launch tendon_calibration.py separately in a terminal:\n\n"
            "  python tendon_calibration.py /dev/tendons\n\n"
            "This GUI will reflect updated angles automatically once\n"
            "the TendonController thread is running."
        )


# ─────────────────────────────────────────────────────────────────────────────
# Main window
# ─────────────────────────────────────────────────────────────────────────────
class BatBotGUI(QMainWindow):
    def __init__(self, sonar_port=None, gps_port=None, tendon_port=None,
                 sim_mode=False, sim_target_m=0.8):
        super().__init__()
        self.sonar_port   = sonar_port
        self.gps_port     = gps_port
        self.tendon_port  = tendon_port
        self.sim_mode     = sim_mode
        self.sim_target_m = sim_target_m

        self.setWindowTitle("BatBot Mission Control")
        self.resize(1400, 860)

        # Workers / state
        self.sonar_worker  = None
        self.gps_worker    = None
        self.tendon_worker = None
        self.sim_workers   = []
        self.signals       = Signals()
        self.signals.new_frame.connect(self._on_sonar_frame)
        self.signals.gps_update.connect(self._on_gps_update)
        self.signals.tendon_angle.connect(self._on_tendon_angle)
        self.signals.error.connect(self._on_error)

        self.recording      = False
        self.gps_running    = False
        self.session_start  = time.time()
        self.raw_buffer     = []          # list of (raw1, raw2, timestamp)
        self.gps_points     = []
        self.enu_origin     = None
        self.gpx_file_count = 0
        self.last_gps       = {}

        self._build_ui()

        # Clock timer
        self.clock_timer = QTimer()
        self.clock_timer.timeout.connect(self._tick_clock)
        self.clock_timer.start(500)

        # Auto-start simulation workers if requested
        if self.sim_mode:
            self._start_sim_workers()

    # ── UI construction ───────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(6, 6, 6, 6)
        root.setSpacing(6)

        # ── Top bar: clock + GPS ──────────────────────────────────────────────
        top = QHBoxLayout()

        clock_box = QGroupBox("System time")
        cl = QVBoxLayout(clock_box)
        self.clock_lbl = QLabel("--:--:--")
        self.clock_lbl.setFont(QFont("Courier New", 22, QFont.Bold))
        cl.addWidget(self.clock_lbl)
        self.date_lbl = QLabel("---")
        self.date_lbl.setStyleSheet("color: gray; font-size: 10px;")
        cl.addWidget(self.date_lbl)
        sess_row = QHBoxLayout()
        sess_row.addWidget(QLabel("Session:"))
        self.sess_lbl = QLabel("00:00:00")
        self.sess_lbl.setFont(QFont("Courier New", 10))
        sess_row.addWidget(self.sess_lbl)
        self.rec_lbl = QLabel("● REC")
        self.rec_lbl.setStyleSheet("color: gray; font-weight: bold;")
        sess_row.addWidget(self.rec_lbl)
        cl.addLayout(sess_row)
        top.addWidget(clock_box, 1)

        gps_box = QGroupBox("GPS  ·  ZED-F9P")
        gl = QGridLayout(gps_box)
        gl.setSpacing(4)
        self._gps_labels = {}
        fields = [
            ('lat',  'Latitude',   0, 0),
            ('lon',  'Longitude',  0, 2),
            ('alt',  'Altitude (m)', 1, 0),
            ('pdop', 'pDOP',        1, 2),
            ('fix',  'Fix type',    2, 0),
            ('ntrip','NTRIP',       2, 2),
        ]
        for key, name, r, c in fields:
            gl.addWidget(QLabel(name + ":"), r, c)
            val = QLabel("---")
            val.setFont(QFont("Courier New", 10))
            gl.addWidget(val, r, c + 1)
            self._gps_labels[key] = val
        top.addWidget(gps_box, 2)
        root.addLayout(top)

        # ── Middle: spectrograms ──────────────────────────────────────────────
        spec_row = QHBoxLayout()
        self.spec_l = SpectrogramWidget("Left pinna")
        self.spec_r = SpectrogramWidget("Right pinna")
        spec_row.addWidget(self.spec_l)
        spec_row.addWidget(self.spec_r)

        spec_info_l = QHBoxLayout()
        self.peak_l_lbl = QLabel("Peak: ---")
        self.frames_l_lbl = QLabel("Frames: 0")
        spec_info_l.addWidget(self.peak_l_lbl)
        spec_info_l.addWidget(self.frames_l_lbl)
        spec_info_r = QHBoxLayout()
        self.peak_r_lbl = QLabel("Peak: ---")
        self.frames_r_lbl = QLabel("Frames: 0")
        spec_info_r.addWidget(self.peak_r_lbl)
        spec_info_r.addWidget(self.frames_r_lbl)

        spec_block = QVBoxLayout()
        spec_block.addLayout(spec_row)

        spec_footer = QHBoxLayout()
        left_info = QGroupBox()
        left_info.setFlat(True)
        lil = QHBoxLayout(left_info)
        lil.addLayout(spec_info_l)
        right_info = QGroupBox()
        right_info.setFlat(True)
        ril = QHBoxLayout(right_info)
        ril.addLayout(spec_info_r)
        spec_footer.addWidget(left_info)
        spec_footer.addWidget(right_info)
        spec_block.addLayout(spec_footer)
        root.addLayout(spec_block, 3)

        self._frame_count = [0, 0]

        # ── Bottom tabs ───────────────────────────────────────────────────────
        tabs = QTabWidget()

        # Tab 1: ENU map
        enu_tab = QWidget()
        enu_layout = QHBoxLayout(enu_tab)
        self.enu_widget = ENUWidget()
        enu_layout.addWidget(self.enu_widget, 3)

        enu_stats = QGroupBox("ENU coordinates")
        esl = QGridLayout(enu_stats)
        self._enu_labels = {}
        for i, (key, name) in enumerate([('e','East (m)'),('n','North (m)'),
                                          ('u','Up (m)'),('pts','Track pts')]):
            esl.addWidget(QLabel(name + ":"), i, 0)
            lbl = QLabel("0.000")
            lbl.setFont(QFont("Courier New", 10))
            esl.addWidget(lbl, i, 1)
            self._enu_labels[key] = lbl
        enu_reset_btn = QPushButton("Reset origin")
        enu_reset_btn.clicked.connect(self._reset_enu)
        esl.addWidget(enu_reset_btn, 4, 0, 1, 2)
        enu_layout.addWidget(enu_stats, 1)
        tabs.addTab(enu_tab, "ENU map")

        # Tab 2: Tendon control
        self.tendon_panel = TendonPanel(NUM_TENDONS)
        tabs.addTab(self.tendon_panel, f"Tendon control ({NUM_TENDONS})")

        # Tab 3: Recording & save
        rec_tab = QWidget()
        rec_layout = QHBoxLayout(rec_tab)

        # Sonar recording
        sonar_rec = QGroupBox("Sonar / chirp recording")
        srl = QVBoxLayout(sonar_rec)
        srl.addWidget(QLabel("Filename prefix:"))
        self.fname_edit = QLineEdit("experiment_01")
        srl.addWidget(self.fname_edit)
        srl.addWidget(QLabel("f low (kHz):"))
        self.flo_spin = QDoubleSpinBox()
        self.flo_spin.setRange(1, 500); self.flo_spin.setValue(30)
        srl.addWidget(self.flo_spin)
        srl.addWidget(QLabel("f high (kHz):"))
        self.fhi_spin = QDoubleSpinBox()
        self.fhi_spin.setRange(1, 500); self.fhi_spin.setValue(100)
        srl.addWidget(self.fhi_spin)
        srl.addWidget(QLabel("Time offset (samples):"))
        self.toff_spin = QSpinBox()
        self.toff_spin.setRange(0, N); self.toff_spin.setValue(4000)
        srl.addWidget(self.toff_spin)
        self.rec_btn = QPushButton("▶  Start recording")
        self.rec_btn.setStyleSheet("font-weight:bold;")
        self.rec_btn.clicked.connect(self._toggle_record)
        srl.addWidget(self.rec_btn)
        self.save_npy_btn = QPushButton("⬇  Save .npy  (timestamped)")
        self.save_npy_btn.clicked.connect(self._save_npy)
        srl.addWidget(self.save_npy_btn)
        self.save_mat_btn = QPushButton("⬇  Save .mat  (MATLAB)")
        self.save_mat_btn.clicked.connect(self._save_mat)
        srl.addWidget(self.save_mat_btn)
        self.sonar_port_edit = QLineEdit(self.sonar_port or "/dev/sonar")
        srl.addWidget(QLabel("Sonar serial port:"))
        srl.addWidget(self.sonar_port_edit)
        self.sonar_connect_btn = QPushButton("Connect sonar")
        self.sonar_connect_btn.clicked.connect(self._connect_sonar)
        srl.addWidget(self.sonar_connect_btn)
        srl.addStretch()
        rec_layout.addWidget(sonar_rec)

        # GPS save
        gps_rec = QGroupBox("GPS recording")
        grl = QVBoxLayout(gps_rec)
        grl.addWidget(QLabel("Run name:"))
        self.grun_edit = QLineEdit("run_01")
        grl.addWidget(self.grun_edit)
        grl.addWidget(QLabel("GPS serial port:"))
        self.gps_port_edit = QLineEdit(self.gps_port or "/dev/gps")
        grl.addWidget(self.gps_port_edit)
        grl.addWidget(QLabel("NTRIP user (blank = disabled):"))
        self.ntrip_user_edit = QLineEdit("")
        grl.addWidget(self.ntrip_user_edit)
        grl.addWidget(QLabel("Mountpoint:"))
        self.mountpoint_edit = QLineEdit("VTTI_SR_RTCM3")
        grl.addWidget(self.mountpoint_edit)
        self.gps_btn = QPushButton("▶  Start GPS")
        self.gps_btn.setStyleSheet("font-weight:bold;")
        self.gps_btn.clicked.connect(self._toggle_gps)
        grl.addWidget(self.gps_btn)
        self.save_gpx_btn = QPushButton("⬇  Save .gpx  (timestamped)")
        self.save_gpx_btn.clicked.connect(self._save_gpx)
        grl.addWidget(self.save_gpx_btn)
        self.gpx_count_lbl = QLabel("Saved segments: 0")
        grl.addWidget(self.gpx_count_lbl)
        grl.addStretch()
        rec_layout.addWidget(gps_rec)

        tabs.addTab(rec_tab, "Recording & save")
        root.addWidget(tabs, 2)

        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready  ·  No serial connections active")

    # ── Clock ─────────────────────────────────────────────────────────────────
    def _tick_clock(self):
        now = datetime.now()
        self.clock_lbl.setText(now.strftime("%H:%M:%S"))
        self.date_lbl.setText(now.strftime("%A  %d %b %Y"))
        elapsed = int(time.time() - self.session_start)
        h, r = divmod(elapsed, 3600)
        m, s = divmod(r, 60)
        self.sess_lbl.setText(f"{h:02d}:{m:02d}:{s:02d}")
        self.rec_lbl.setStyleSheet(
            "color: red; font-weight: bold;" if self.recording else "color: gray; font-weight: bold;"
        )

    # ── Sonar callbacks ───────────────────────────────────────────────────────
    def _on_sonar_frame(self, raw1: bytes, raw2: bytes):
        toff = self.toff_spin.value()
        flo  = self.flo_spin.value()
        fhi  = self.fhi_spin.value()

        S1, f1, t1 = compute_spectrogram(raw1, toff)
        S2, f2, t2 = compute_spectrogram(raw2, toff)

        # Trim to display freq bounds
        m1 = (f1 >= flo) & (f1 <= fhi)
        m2 = (f2 >= flo) & (f2 <= fhi)
        self.spec_l.update_data(S1[m1, :], f1[m1], t1)
        self.spec_r.update_data(S2[m2, :], f2[m2], t2)

        self._frame_count[0] += 1
        self._frame_count[1] += 1
        self.peak_l_lbl.setText(f"Peak: {self.spec_l.peak_dB:.1f} dB")
        self.peak_r_lbl.setText(f"Peak: {self.spec_r.peak_dB:.1f} dB")
        self.frames_l_lbl.setText(f"Frames: {self._frame_count[0]}")
        self.frames_r_lbl.setText(f"Frames: {self._frame_count[1]}")

        if self.recording:
            self.raw_buffer.append((raw1, raw2, datetime.utcnow().isoformat()))

    # ── GPS callbacks ─────────────────────────────────────────────────────────
    def _on_gps_update(self, d: dict):
        self.last_gps = d
        fix_names = {0: "No fix", 1: "Dead reck.", 2: "2D fix",
                     3: "3D fix", 4: "GNSS+DR", 5: "Time only"}
        self._gps_labels['lat'].setText(f"{d['lat']:.7f}°")
        self._gps_labels['lon'].setText(f"{d['lon']:.7f}°")
        self._gps_labels['alt'].setText(f"{d['alt']:.2f}")
        self._gps_labels['pdop'].setText(f"{d['pdop']:.2f}")
        self._gps_labels['fix'].setText(fix_names.get(d['fix'], str(d['fix'])))
        if d['fix'] >= 3:
            self._gps_labels['fix'].setStyleSheet("color: green; font-weight:bold;")
        else:
            self._gps_labels['fix'].setStyleSheet("color: orange;")

        dE, dN = self.enu_widget.add_point(d['lat'], d['lon'], d['alt'])
        self._enu_labels['e'].setText(f"{dE:.3f}")
        self._enu_labels['n'].setText(f"{dN:.3f}")
        self._enu_labels['u'].setText(f"{d['alt']:.3f}")
        self._enu_labels['pts'].setText(str(len(self.enu_widget.east_pts)))

        self.gps_points.append(d)

    def _on_tendon_angle(self, idx: int, angle: float):
        self.tendon_panel.set_angle(idx, angle)

    def _start_sim_workers(self):
        """Start all three simulation workers and update UI to reflect sim mode."""
        sonar_sim = SonarSimWorker(
            self.signals, sim_target_m=self.sim_target_m, frame_rate_hz=4.0
        )
        gps_sim   = GPSSimWorker(self.signals, rate_hz=2.0)
        tendon_sim = TendonSimWorker(self.signals, rate_hz=10.0)

        for w in (sonar_sim, gps_sim, tendon_sim):
            w.start()
            self.sim_workers.append(w)

        # Mark GPS as running so toggle button stays consistent
        self.gps_running = True
        self.gps_btn.setText("■  Stop GPS (sim)")
        self.gps_btn.setStyleSheet("font-weight:bold; color:#EF9F27;")
        self._gps_labels['ntrip'].setText("SIM")
        self._gps_labels['ntrip'].setStyleSheet("color:#EF9F27; font-weight:bold;")
        self._gps_labels['fix'].setText("3D fix (sim)")
        self._gps_labels['fix'].setStyleSheet("color:#EF9F27; font-weight:bold;")

        # Mark recording active automatically so spectrograms stream
        self.recording = True
        self.rec_btn.setText("■  Stop recording (sim)")
        self.rec_btn.setStyleSheet("font-weight:bold; color:#EF9F27;")

        self.status_bar.showMessage(
            f"SIMULATION MODE  ·  sonar @ 4 Hz  ·  GPS @ 2 Hz  ·  "
            f"target at {self.sim_target_m:.2f} m  ·  12 tendons sweeping"
        )

    def _stop_sim_workers(self):
        for w in self.sim_workers:
            w.stop()
            w.wait()
        self.sim_workers.clear()

    def _on_error(self, msg: str):
        self.status_bar.showMessage(f"Error: {msg}")

    # ── Controls ──────────────────────────────────────────────────────────────
    def _toggle_record(self):
        self.recording = not self.recording
        if self.recording:
            self.rec_btn.setText("■  Stop recording")
            self.rec_btn.setStyleSheet("font-weight:bold; color:red;")
            self.status_bar.showMessage("Recording sonar data...")
        else:
            self.rec_btn.setText("▶  Start recording")
            self.rec_btn.setStyleSheet("font-weight:bold;")
            self.status_bar.showMessage(
                f"Recording stopped  ·  {len(self.raw_buffer)} frames buffered"
            )

    def _toggle_gps(self):
        self.gps_running = not self.gps_running
        if self.gps_running:
            self.gps_btn.setText("■  Stop GPS")
            self.gps_btn.setStyleSheet("font-weight:bold; color:red;")
            self._gps_labels['ntrip'].setText("Connecting...")
            self._start_gps_worker()
        else:
            self.gps_btn.setText("▶  Start GPS")
            self.gps_btn.setStyleSheet("font-weight:bold;")
            if self.gps_worker:
                self.gps_worker.stop()
                self.gps_worker = None
            self._gps_labels['ntrip'].setText("Disconnected")

    def _connect_sonar(self):
        port = self.sonar_port_edit.text().strip()
        if self.sonar_worker:
            self.sonar_worker.stop()
            self.sonar_worker.wait()
        self.sonar_worker = SonarWorker(port, self.signals)
        self.sonar_worker.start()
        self.status_bar.showMessage(f"Sonar worker started on {port}")

    def _start_gps_worker(self):
        port   = self.gps_port_edit.text().strip()
        user   = self.ntrip_user_edit.text().strip() or None
        mount  = self.mountpoint_edit.text().strip()
        self.gps_worker = GPSWorker(
            port, self.signals,
            ntrip_user=user, mountpoint=mount
        )
        self.gps_worker.start()
        self.status_bar.showMessage(f"GPS worker started on {port}")

    def _reset_enu(self):
        self.enu_widget.reset()

    # ── Save helpers ──────────────────────────────────────────────────────────
    def _timestamped_name(self, prefix, ext):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"{prefix}_{ts}{ext}"

    def _save_npy(self):
        if not self.raw_buffer:
            QMessageBox.warning(self, "No data", "No frames recorded yet.")
            return
        prefix = self.fname_edit.text().strip() or "experiment"
        os.makedirs("data", exist_ok=True)
        fname = os.path.join("data", self._timestamped_name(prefix, ".npy"))
        with open(fname, 'wb') as fd:
            for raw1, raw2, ts in self.raw_buffer:
                np.save(fd, np.frombuffer(raw1, dtype=np.uint8))
                np.save(fd, np.frombuffer(raw2, dtype=np.uint8))
        self.status_bar.showMessage(f"Saved {len(self.raw_buffer)} frames → {fname}")

    def _save_mat(self):
        if not self.raw_buffer:
            QMessageBox.warning(self, "No data", "No frames recorded yet.")
            return
        prefix = self.fname_edit.text().strip() or "experiment"
        os.makedirs("data", exist_ok=True)
        fname = os.path.join("data", self._timestamped_name(prefix, ".mat"))
        all_ch1 = np.stack([bin2dec(r1) for r1, _, _ in self.raw_buffer])
        all_ch2 = np.stack([bin2dec(r2) for _, r2, _ in self.raw_buffer])
        times   = [ts for _, _, ts in self.raw_buffer]
        savemat(fname, {'ch1': all_ch1, 'ch2': all_ch2,
                        'Fs': FS, 'N_chirp': N_CHIRP,
                        'timestamps': times})
        self.status_bar.showMessage(f"Saved .mat → {fname}")

    def _save_gpx(self):
        if not self.gps_points:
            QMessageBox.warning(self, "No GPS data", "No GPS points recorded.")
            return
        try:
            import gpxpy
            from gpxpy.gpx import GPX, GPXTrack, GPXTrackSegment, GPXTrackPoint
            gpx = GPX()
            gpx.name = "BatBot 7"
            gpx.description = "GPS data from ZED-F9P"
            track = GPXTrack()
            gpx.tracks.append(track)
            seg = GPXTrackSegment()
            track.segments.append(seg)
            for pt in self.gps_points:
                seg.points.append(GPXTrackPoint(
                    latitude=pt['lat'], longitude=pt['lon'],
                    elevation=pt['alt'], position_dilution=pt['pdop']
                ))
            prefix = self.grun_edit.text().strip() or "run"
            os.makedirs("data", exist_ok=True)
            self.gpx_file_count += 1
            fname = os.path.join(
                "data",
                f"{prefix}_GPS_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                f"_part{self.gpx_file_count}.gpx"
            )
            with open(fname, 'w') as f:
                f.write(gpx.to_xml())
            self.gpx_count_lbl.setText(f"Saved segments: {self.gpx_file_count}")
            self.status_bar.showMessage(f"Saved GPX → {fname}")
            self.gps_points.clear()
        except ImportError:
            QMessageBox.warning(self, "Missing library", "gpxpy not installed.\npip install gpxpy")

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def closeEvent(self, event):
        self._stop_sim_workers()
        if self.sonar_worker:
            self.sonar_worker.stop()
            self.sonar_worker.wait()
        if self.gps_worker:
            self.gps_worker.stop()
            self.gps_worker.wait()
        event.accept()


# ─────────────────────────────────────────────────────────────────────────────
# Entry point
# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="BatBot Mission Control GUI")
    parser.add_argument("--sonar",      default=None,  help="Sonar serial port  (e.g. /dev/ttyUSB0)")
    parser.add_argument("--gps",        default=None,  help="GPS serial port    (e.g. /dev/ttyUSB1)")
    parser.add_argument("--tendons",    default=None,  help="Tendon serial port (e.g. /dev/ttyUSB2)")
    parser.add_argument("--sim",        action="store_true",
                        help="Run in simulation mode (no serial hardware needed)")
    parser.add_argument("--sim-target", type=float, default=0.8, metavar="METRES",
                        help="One-way target range for simulated echo (default: 0.8 m)")
    args = parser.parse_args()

    # pyqtgraph dark theme
    pg.setConfigOption('background', '#0f0f14')
    pg.setConfigOption('foreground', '#cccccc')
    pg.setConfigOptions(antialias=True)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette
    palette = QPalette()
    palette.setColor(QPalette.Window,          QColor(30, 30, 38))
    palette.setColor(QPalette.WindowText,      QColor(210, 210, 210))
    palette.setColor(QPalette.Base,            QColor(22, 22, 30))
    palette.setColor(QPalette.AlternateBase,   QColor(36, 36, 46))
    palette.setColor(QPalette.ToolTipBase,     QColor(50, 50, 60))
    palette.setColor(QPalette.ToolTipText,     QColor(210, 210, 210))
    palette.setColor(QPalette.Text,            QColor(210, 210, 210))
    palette.setColor(QPalette.Button,          QColor(45, 45, 58))
    palette.setColor(QPalette.ButtonText,      QColor(210, 210, 210))
    palette.setColor(QPalette.BrightText,      QColor(255, 80, 80))
    palette.setColor(QPalette.Link,            QColor(66, 153, 225))
    palette.setColor(QPalette.Highlight,       QColor(56, 139, 220))
    palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))
    app.setPalette(palette)

    win = BatBotGUI(
        sonar_port=args.sonar,
        gps_port=args.gps,
        tendon_port=args.tendons,
        sim_mode=args.sim,
        sim_target_m=args.sim_target,
    )
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
