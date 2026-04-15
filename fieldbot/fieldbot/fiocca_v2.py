import sys
import numpy as np
from datetime import datetime

from PyQt6.QtWidgets import *
from PyQt6.QtCore import QTimer, QThread, pyqtSignal
import pyqtgraph as pg
from scipy import signal


# ---------------- GPS THREAD ---------------- #
class GPSThread(QThread):
    new_data = pyqtSignal(float, float)

    def run(self):
        t = 0
        while True:
            lat = 38.0 + np.sin(t) * 0.0001
            lon = -78.7 + np.cos(t) * 0.0001
            self.new_data.emit(lat, lon)
            t += 0.1
            self.msleep(500)


# ---------------- MAIN WINDOW ---------------- #
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Batbot System")
        self.setGeometry(100, 100, 1400, 900)

        self.setStyleSheet("""
        QMainWindow { background-color: #121212; }
        QLabel { color: white; font-size: 14px; }
        QPushButton {
            background-color: #1f6feb;
            border-radius: 8px;
            padding: 6px;
            color: white;
        }
        """)

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.tabs.addTab(self.sonar_tab(), "Sonar + GPS")
        self.tabs.addTab(self.tendon_tab(), "Tendons")

        # GPS thread
        self.gps_thread = GPSThread()
        self.gps_thread.new_data.connect(self.update_gps)
        self.gps_thread.start()

    # ---------------- SONAR TAB ---------------- #
    def sonar_tab(self):
        widget = QWidget()
        layout = QHBoxLayout(widget)

        # LEFT PANEL
        left = QVBoxLayout()

        self.spec_left = pg.PlotWidget(title="Left Pinna")
        self.img_left = pg.ImageItem()
        self.spec_left.addItem(self.img_left)

        self.spec_right = pg.PlotWidget(title="Right Pinna")
        self.img_right = pg.ImageItem()
        self.spec_right.addItem(self.img_right)

        # Labels
        for plot in [self.spec_left, self.spec_right]:
            plot.setLabel('left', 'Frequency (kHz)')
            plot.setLabel('bottom', 'Time (ms)')

        left.addWidget(self.spec_left)
        left.addWidget(self.spec_right)

        # BUTTONS
        btns = QHBoxLayout()
        self.start_btn = QPushButton("Start")
        self.stop_btn = QPushButton("Stop")
        self.save_btn = QPushButton("Save Data")

        btns.addWidget(self.start_btn)
        btns.addWidget(self.stop_btn)
        btns.addWidget(self.save_btn)

        left.addLayout(btns)

        # CHIRP CONTROLS
        form = QFormLayout()

        self.f0 = QSpinBox(); self.f0.setValue(100000)
        self.f1 = QSpinBox(); self.f1.setValue(30000)
        self.T = QDoubleSpinBox(); self.T.setValue(0.003)

        form.addRow("Start Freq (Hz)", self.f0)
        form.addRow("End Freq (Hz)", self.f1)
        form.addRow("Duration (s)", self.T)

        left.addLayout(form)

        # RIGHT PANEL
        right = QVBoxLayout()

        # GPS MAP
        self.map_plot = pg.PlotWidget(title="ENU Position")
        self.map_plot.setXRange(-10, 10)
        self.map_plot.setYRange(-10, 10)
        self.map_point = self.map_plot.plot([0], [0], pen=None, symbol='o')

        right.addWidget(self.map_plot)

        # GPS INFO PANEL
        self.gps_label = QLabel("Lat: -- | Lon: --")
        right.addWidget(self.gps_label)

        # SYSTEM TIME
        self.time_label = QLabel("Time: --:--:--")
        right.addWidget(self.time_label)

        layout.addLayout(left, 2)
        layout.addLayout(right, 1)

        # TIMERS
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_sonar)

        self.clock = QTimer()
        self.clock.timeout.connect(self.update_time)
        self.clock.start(1000)

        self.start_btn.clicked.connect(lambda: self.timer.start(100))
        self.stop_btn.clicked.connect(self.timer.stop)
        self.save_btn.clicked.connect(self.save_data)

        return widget

    # ---------------- SONAR UPDATE ---------------- #
    def update_sonar(self):
        Fs = 1_000_000
        c = 343  # speed of sound

        f0 = self.f0.value()
        f1 = self.f1.value()
        T = self.T.value()

        t = np.linspace(0, T, int(Fs*T))

        chirp = signal.chirp(t, f0, T, f1)

        sig_L = chirp + np.random.normal(0, 0.2, len(chirp))
        sig_R = chirp + np.random.normal(0, 0.2, len(chirp))

        f, t_spec, Sxx_L = signal.spectrogram(sig_L, Fs)
        _, _, Sxx_R = signal.spectrogram(sig_R, Fs)

        # Convert axes
        f_khz = f / 1000
        t_ms = t_spec * 1000

        # Distance conversion (cm)
        dist_cm = (c * t_spec / 2) * 100

        # Apply bat-relevant limits
        mask = (f_khz >= 20) & (f_khz <= 120)

        Sxx_L = Sxx_L[mask, :]
        Sxx_R = Sxx_R[mask, :]
        f_khz = f_khz[mask]

        self.img_left.setImage(10*np.log10(Sxx_L + 1e-12))
        self.img_right.setImage(10*np.log10(Sxx_R + 1e-12))

    # ---------------- GPS ---------------- #
    def update_gps(self, lat, lon):
        self.gps_label.setText(f"Lat: {lat:.6f} | Lon: {lon:.6f}")

        # Fake ENU mapping
        x = (lon + 78.7) * 10000
        y = (lat - 38.0) * 10000

        self.map_point.setData([x], [y])

    # ---------------- TIME ---------------- #
    def update_time(self):
        now = datetime.now().strftime("%H:%M:%S")
        self.time_label.setText(f"Time: {now}")

    # ---------------- SAVE ---------------- #
    def save_data(self):
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"sonar_{now}.npy"

        data = np.random.rand(1000)
        np.save(filename, data)

        self.time_label.setText(f"Saved: {filename}")

    # ---------------- TENDON TAB ---------------- #
    def tendon_tab(self):
        widget = QWidget()
        layout = QGridLayout(widget)

        self.motor_labels = []

        for i in range(12):
            label = QLabel(f"Motor {i+1}: 0°")
            layout.addWidget(label, i // 4, i % 4)
            self.motor_labels.append(label)

        self.tendon_timer = QTimer()
        self.tendon_timer.timeout.connect(self.update_motors)
        self.tendon_timer.start(500)

        return widget

    def update_motors(self):
        for i, label in enumerate(self.motor_labels):
            angle = np.random.randint(-90, 90)
            label.setText(f"Motor {i+1}: {angle}°")


# ---------------- RUN ---------------- #
app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec())