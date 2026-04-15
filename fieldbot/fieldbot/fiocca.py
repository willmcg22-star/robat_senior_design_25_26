import sys
import numpy as np
from PyQt6.QtWidgets import *
from PyQt6.QtCore import QTimer, QThread, pyqtSignal
import pyqtgraph as pg
from scipy import signal

# ---------------- GPS THREAD ---------------- #
class GPSThread(QThread):
    new_coord = pyqtSignal(float, float)

    def run(self):
        # Simulated GPS (replace with bb_gps2)
        t = 0
        while True:
            x = np.cos(t) * 5
            y = np.sin(t) * 5
            self.new_coord.emit(x, y)
            t += 0.1
            self.msleep(200)

# ---------------- MAIN WINDOW ---------------- #
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Batbot Control System")
        self.setGeometry(100, 100, 1300, 850)

        self.setStyleSheet("background-color: #121212; color: white;")

        self.tabs = QTabWidget()
        self.setCentralWidget(self.tabs)

        self.tabs.addTab(self.sonar_tab(), "Sonar + GPS")
        self.tabs.addTab(self.tendon_tab(), "Tendons")

        # Start GPS
        self.gps_thread = GPSThread()
        self.gps_thread.new_coord.connect(self.update_map)
        self.gps_thread.start()

    # ---------------- SONAR + GPS TAB ---------------- #
    def sonar_tab(self):
        widget = QWidget()
        layout = QHBoxLayout(widget)

        # LEFT: Spectrogram
        left_panel = QVBoxLayout()

        self.spec_plot = pg.PlotWidget(title="Spectrogram")
        self.img = pg.ImageItem()
        self.spec_plot.addItem(self.img)

        left_panel.addWidget(self.spec_plot)

        # Controls
        control_layout = QHBoxLayout()

        self.start_btn = QPushButton("Start")
        self.stop_btn = QPushButton("Stop")

        control_layout.addWidget(self.start_btn)
        control_layout.addWidget(self.stop_btn)

        left_panel.addLayout(control_layout)

        # Chirp settings
        chirp_layout = QFormLayout()

        self.f0_input = QSpinBox()
        self.f0_input.setValue(100000)

        self.f1_input = QSpinBox()
        self.f1_input.setValue(30000)

        self.duration_input = QDoubleSpinBox()
        self.duration_input.setValue(0.003)

        chirp_layout.addRow("Start Freq", self.f0_input)
        chirp_layout.addRow("End Freq", self.f1_input)
        chirp_layout.addRow("Duration", self.duration_input)

        left_panel.addLayout(chirp_layout)

        # RIGHT: GPS MAP
        right_panel = QVBoxLayout()

        self.map_plot = pg.PlotWidget(title="ENU Position")
        self.map_plot.setXRange(-10, 10)
        self.map_plot.setYRange(-10, 10)

        self.map_point = self.map_plot.plot([0], [0], pen=None, symbol='o')

        right_panel.addWidget(self.map_plot)

        # Layout split
        layout.addLayout(left_panel, 2)
        layout.addLayout(right_panel, 1)

        # Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_spectrogram)

        self.start_btn.clicked.connect(lambda: self.timer.start(100))
        self.stop_btn.clicked.connect(self.timer.stop)

        return widget

    # ---------------- SPECTROGRAM ---------------- #
    def update_spectrogram(self):
        Fs = 1_000_000

        f0 = self.f0_input.value()
        f1 = self.f1_input.value()
        T = self.duration_input.value()

        t = np.linspace(0, T, int(Fs*T))

        chirp = signal.chirp(t, f0, T, f1)

        noise = np.random.normal(0, 0.2, len(chirp))
        echo = chirp + noise

        f, t_spec, Sxx = signal.spectrogram(echo, Fs)

        self.img.setImage(10*np.log10(Sxx + 1e-12))

    # ---------------- GPS UPDATE ---------------- #
    def update_map(self, x, y):
        self.map_point.setData([x], [y])

    # ---------------- TENDON TAB ---------------- #
    def tendon_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)

        self.labels = []

        for i in range(5):
            label = QLabel(f"Motor {i+1}: 0°")
            self.labels.append(label)
            layout.addWidget(label)

        # Simulated updates (replace with TendonController reads)
        self.tendon_timer = QTimer()
        self.tendon_timer.timeout.connect(self.update_tendons)
        self.tendon_timer.start(500)

        return widget

    def update_tendons(self):
        for i, label in enumerate(self.labels):
            angle = np.random.randint(-90, 90)
            label.setText(f"Motor {i+1}: {angle}°")


# ---------------- RUN ---------------- #
app = QApplication(sys.argv)
window = MainWindow()
window.show()
sys.exit(app.exec())