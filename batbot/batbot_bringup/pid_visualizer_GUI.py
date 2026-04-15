import sys
import os

# Add the package root to Python path
package_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
sys.path.append(package_root)

from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                           QHBoxLayout, QLabel, QSpinBox, QLineEdit, QPushButton,
                           QGroupBox, QGridLayout)
from PyQt6.QtCore import QTimer, Qt
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np
import time


# Direct import from the same directory
from batbot_bringup.bb_tendons.TendonController import TendonController
from collections import deque

class PIDVisualizer(QMainWindow):
    def __init__(self, port_name=""):
        super().__init__()
        self.setWindowTitle("PID Step Response Visualizer")
        self.setGeometry(100, 100, 1200, 800)
        
        # Data for plotting
        self.time_data = deque(maxlen=200)
        self.angle_data = deque(maxlen=200)
        self.target_angle = 0
        self.is_recording = False
        self.start_time = 0
        
        # Create main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Create control panel
        control_group = QGroupBox("Controls")
        control_layout = QGridLayout()
        
        # Motor selection
        control_layout.addWidget(QLabel("Motor ID:"), 0, 0)
        self.motor_id = QSpinBox()
        self.motor_id.setRange(0, 7)
        self.motor_id.setValue(0)
        control_layout.addWidget(self.motor_id, 0, 1)
        
        # PID Parameters
        control_layout.addWidget(QLabel("P:"), 1, 0)
        self.kp = QLineEdit("100")
        control_layout.addWidget(self.kp, 1, 1)
        
        control_layout.addWidget(QLabel("I:"), 2, 0)
        self.ki = QLineEdit("0")
        control_layout.addWidget(self.ki, 2, 1)
        
        control_layout.addWidget(QLabel("D:"), 3, 0)
        self.kd = QLineEdit("10")
        control_layout.addWidget(self.kd, 3, 1)
        
        # Step input controls
        control_layout.addWidget(QLabel("Step Size (°):"), 4, 0)
        self.step_size = QLineEdit("90")
        control_layout.addWidget(self.step_size, 4, 1)
        
        # Buttons
        self.apply_pid_btn = QPushButton("Apply PID")
        self.apply_pid_btn.clicked.connect(self.apply_pid)
        control_layout.addWidget(self.apply_pid_btn, 5, 0)
        
        self.start_test_btn = QPushButton("Start Test")
        self.start_test_btn.clicked.connect(self.start_test)
        control_layout.addWidget(self.start_test_btn, 5, 1)
        
        self.reset_btn = QPushButton("Reset")
        self.reset_btn.clicked.connect(self.reset_plot)
        control_layout.addWidget(self.reset_btn, 5, 2)
        
        control_group.setLayout(control_layout)
        main_layout.addWidget(control_group)
        
        # Create plot
        self.fig = Figure(figsize=(8, 6))
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Step Response")
        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("Angle (degrees)")
        self.ax.grid(True)
        
        # Add plot to GUI
        self.canvas = FigureCanvas(self.fig)
        main_layout.addWidget(self.canvas)
        
        # Initialize controller in test mode
        self.tc = TendonController(port_name=port_name)
        
        # Status label
        self.status_label = QLabel("")
        self.statusBar().addWidget(self.status_label)
        
        # Setup update timer
        self.timer = QTimer()
        self.steady_state_timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.setInterval(50)  # 20Hz update rate
    
    def apply_pid(self):
        try:
            motor_id = self.motor_id.value()
            kp = float(self.kp.text())
            ki = float(self.ki.text())
            kd = float(self.kd.text())
            
            self.tc.setMotorPID(motor_id, kp, ki, kd)
            self.status_label.setText("PID parameters applied successfully!")
        except ValueError:
            self.status_label.setText("Error: Please enter valid numbers")
        except Exception as e:
            self.status_label.setText(f"Error: {str(e)}")
    
    def start_test(self):
        try:
            self.reset_plot()
            motor_id = self.motor_id.value()
            step_size = float(self.step_size.text())
            
            # Start recording
            self.is_recording = True
            self.start_time = time.time()
            self.target_angle = step_size
            
            # Move motor to new position
            self.tc.setNewZero(motor_id)
            print(f'Setting Angle to {int(step_size)}')
            self.tc.writeMotorAbsoluteAngle(motor_id, int(step_size))
            
            # Start update timer
            self.timer.start()
            
            self.status_label.setText("Test started")
        except ValueError:
            self.status_label.setText("Error: Please enter valid numbers")
        except Exception as e:
            self.status_label.setText(f"Error: {str(e)}")

    def check_steady_state(self, data):
        if len(data) < 50:
            return False
        
        window_data = list(data)[-50:]
        mean = np.mean(window_data)
        std = np.std(window_data)

        return (std < 0.01)

    def update_plot(self):
        if not self.is_recording:
            self.timer.stop()
            return
            
        try:
            current_time = time.time() - self.start_time
            current_angle = self.tc.readMotorAngle(self.motor_id.value())
            
            self.time_data.append(current_time)
            self.angle_data.append(current_angle)
            
            # Update plot
            self.ax.clear()
            self.ax.plot(list(self.time_data), list(self.angle_data), 'b-', label='Actual')
            self.ax.axhline(y=self.target_angle, color='r', linestyle='--', label='Target')
            self.ax.set_title("Step Response")
            self.ax.set_xlabel("Time (seconds)")
            self.ax.set_ylabel("Angle (degrees)")
            self.ax.grid(True)
            self.ax.legend()
            
            # Set reasonable y-axis limits
            self.ax.set_ylim([0, max(self.target_angle * 1.2, max(self.angle_data) * 1.1)])
            self.ax.set_xlim([0, current_time * 1.1])

            is_steady_state = self.check_steady_state(self.angle_data)
            
            # Stop recording after 2 seconds
            if current_time >= 10 or is_steady_state:
                self.is_recording = False
                self.timer.stop()
            
            self.canvas.draw()
            
        except Exception as e:
            print(f"Error updating plot: {e}")
            self.is_recording = False
            self.timer.stop()
    
    def reset_plot(self):
        self.is_recording = False
        self.timer.stop()
        self.steady_state_timer.stop()
        self.time_data.clear()
        self.angle_data.clear()
        self.ax.clear()
        self.ax.set_title("Step Response")
        self.ax.set_xlabel("Time (seconds)")
        self.ax.set_ylabel("Angle (degrees)")
        self.ax.grid(True)
        self.canvas.draw()

def main():
    app = QApplication(sys.argv)

    port_name = ""
    if len(sys.argv) == 2:
        port_name = sys.argv[1]

    window = PIDVisualizer(port_name=port_name)
    window.show()
    sys.exit(app.exec())  # Note: exec() instead of exec_() in PyQt6

if __name__ == "__main__":
    main() 