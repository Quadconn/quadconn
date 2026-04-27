### MAIN ###

import sys
import os
import datetime
import time
import matplotlib
matplotlib.use('Agg') # Force the "Silent" backend
import matplotlib.pyplot as plt

# PyQt6 UI Components
from PyQt6.QtCore import Qt, QPoint, pyqtSignal, QTimer, QPointF
from PyQt6.QtWidgets import (QApplication, QLabel, QVBoxLayout, QHBoxLayout,
                             QWidget, QGridLayout, QPushButton, QLineEdit, QSlider)
from PyQt6.QtGui import QPainter, QPen, QColor, QBrush

# Custom Imports from your modular files
from configurations import *
from workers import VideoThread, TelemetryReceiver, AudioReceiveThread, AudioSendThread, ControllerThread
from motor_diagnostics import MOTOR_COUNT

class LidarMapWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedSize(780, 350)
        self.grid_map = None 
        self.is_human_detected = False

    def update_grid(self, grid_map):
        """Receives the persistent numpy array from the SlamProcessor."""
        self.grid_map = grid_map
        self.update()

    def set_human_status(self, is_human):
        """Updates the visual state if the YOLO worker detects a person."""
        self.is_human_detected = is_human

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(event.rect(), QColor(10, 10, 10)) # Darker background
        
        if self.grid_map is None:
            return

        rows, cols = self.grid_map.shape
        cell_w = self.width() / cols
        cell_h = self.height() / rows

        # Optimization: We only draw pixels that are likely walls (> 0.6 probability)
        # or clear floors (< 0.4 probability)
        for y in range(0, rows, 1): # You can increase step to 2 for performance
            for x in range(0, cols, 1):
                prob = self.grid_map[y, x]
                
                if prob > 0.6: # It's a wall!
                    color = QColor(255, 0, 0) if self.is_human_detected else QColor(0, 255, 0)
                    painter.fillRect(int(x * cell_w), int(y * cell_h), 
                                     int(cell_w)+1, int(cell_h)+1, color)
                elif prob < 0.4: # It's floor!
                    painter.fillRect(int(x * cell_w), int(y * cell_h), 
                                     int(cell_w)+1, int(cell_h)+1, QColor(30, 30, 30))

## Main User Interface ##
class QuadconnDashboard(QWidget):
    # Integrates video stream, manual network configuration, 12-motor telemetry grid, and two-way audio
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quadconn Senior Design Dashboard")
        self.setFixedSize(1200, 800)
        self.setStyleSheet("background-color: #0A0A0A; color: #FFFFFF;")
        
        # Main Layout
        main_layout = QHBoxLayout()
        main_layout.setContentsMargins(10, 10, 10, 10)

        # Left Column (Video + Lidar)
        left_column = QVBoxLayout()
        left_column.setSpacing(20)

        left_column.addStretch()
        
        # Live Video Display Label
        self.video_label = QLabel("PASTE YOUR TAILSCALE IP & START SYSTEM")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setFixedSize(780, 350)
        self.video_label.setStyleSheet("border: 2px solid #333; background-color: black;")

        # Radar Widget
        left_column.addWidget(self.video_label, alignment=Qt.AlignmentFlag.AlignCenter)
        self.lidar_map = LidarMapWidget()
        left_column.addWidget(self.lidar_map, alignment=Qt.AlignmentFlag.AlignCenter)
        left_column.addStretch()
        main_layout.addLayout(left_column)
        
        # Sidebar Container
        self.sidebar = QVBoxLayout()
        self.sidebar.setSpacing(8)
        
        # Configuration Section
        self.sidebar.addWidget(QLabel("YOUR TAILSCALE IP:"))
        self.ip_input = QLineEdit()
        self.ip_input.setPlaceholderText("Paste 100.x.x.x address here")
        self.ip_input.setStyleSheet("background-color: #222; border: 1px solid #555; padding: 5px; color: #00FF00;")
        self.sidebar.addWidget(self.ip_input)
        
        self.btn_start = QPushButton("START ROBOT COMMS")
        self.btn_start.setStyleSheet("background-color: #004400; color: white; padding: 10px; font-weight: bold;")
        self.btn_start.clicked.connect(self.restart_hardware_action)
        self.sidebar.addWidget(self.btn_start)

        # Audio Control Section
        self.sidebar.addWidget(QLabel("AUDIO CONTROLS:"))
        self.btn_mic = QPushButton("MIC: UNMUTED")
        self.btn_mic.setStyleSheet("background-color: #222; border: 1px solid #00FF00; color: #00FF00; padding: 8px;")
        self.btn_mic.clicked.connect(self.toggle_mic)
        self.sidebar.addWidget(self.btn_mic)

        self.sidebar.addWidget(QLabel("SPEAKER VOLUME:"))
        self.slider_vol = QSlider(Qt.Orientation.Horizontal)
        self.slider_vol.setRange(0, 100); self.slider_vol.setValue(80)
        self.slider_vol.valueChanged.connect(self.update_speaker_volume)
        self.sidebar.addWidget(self.slider_vol)

        # AI Control Button
        self.btn_ai = QPushButton("PERSON DETECTION: READY")
        self.btn_ai.setStyleSheet("border: 1px solid #00FF00; color: #00FF00; padding: 10px;")
        self.btn_ai.clicked.connect(self.toggle_ai)
        self.sidebar.addWidget(self.btn_ai)

        # Human Detection Function
        self.lbl_ai_proximity = QLabel("SCANNING...")
        self.lbl_ai_proximity.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_ai_proximity.setStyleSheet("background-color: #111; border: 1px solid #555; color: #555; padding: 5px; font-weight: bold;")
        self.sidebar.addWidget(self.lbl_ai_proximity)

        # Status Header (Diagnostics)
        self.status_header = QVBoxLayout()
        
        # Row 1: Battery and Power
        row1 = QHBoxLayout()
        self.lbl_volt = QLabel("BATT: -- V")
        self.lbl_power = QLabel("PWR: -- W")
        row1.addWidget(self.lbl_volt)
        row1.addWidget(self.lbl_power)
        self.status_header.addLayout(row1)
        
        # Row 2: Connection and Health
        row2 = QHBoxLayout()
        self.lbl_status = QLabel("OFFLINE")
        self.lbl_status.setStyleSheet("color: #FF0000; font-weight: bold; border: 1px solid #FF0000; padding: 4px;")
        
        self.lbl_health = QLabel("SYSTEM: OK")
        self.lbl_health.setStyleSheet("color: #00FF00; font-weight: bold;")
        
        row2.addWidget(self.lbl_status)
        row2.addWidget(self.lbl_health)
        self.status_header.addLayout(row2)
        
        self.sidebar.addLayout(self.status_header)

        # Alert Banner
        self.lbl_alert_banner = QLabel("SYSTEM HEALTHY")
        self.lbl_alert_banner.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.lbl_alert_banner.setFixedHeight(40)
        self.lbl_alert_banner.setStyleSheet("background-color: #002200; color: #00FF00; font-weight: bold; border: 1px solid #00FF00;")
        self.sidebar.insertWidget(0, self.lbl_alert_banner) 
        
        self.last_popup_time = 0

        # Enable Controller Button
        self.btn_controller = QPushButton("ENABLE GAMEPAD")
        self.btn_controller.setStyleSheet("background-color: #222; border: 1px solid #555; color: #FFF; padding: 10px;")
        self.btn_controller.clicked.connect(self.toggle_controller)
        self.sidebar.addWidget(self.btn_controller)

        # Record Function
        self.btn_record = QPushButton("START RECORDING")
        self.btn_record.setStyleSheet("background-color: #222; border: 1px solid #FF0000; color: #FF0000; padding: 10px; font-weight: bold;")
        self.btn_record.clicked.connect(self.toggle_recording)
        self.sidebar.addWidget(self.btn_record)
        
        # 12-Motor Grid
        self.grid = QGridLayout()
        self.grid.setSpacing(4)
        self.motor_labels = []
        for i in range(MOTOR_COUNT):
            lbl = QLabel(f"M{i+1}\nWAITING...")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("border: 1px solid #444; font-size: 10px; background-color: #111;")
            self.grid.addWidget(lbl, i // 3, i % 3)
            self.motor_labels.append(lbl)
        self.sidebar.addLayout(self.grid)

        # Reset Recon Map Button
        self.btn_reset_map = QPushButton("RESET RECON MAP")
        self.btn_reset_map.setStyleSheet("background-color: #222; border: 1px solid #555; color: #FFF; padding: 10px; font-weight: bold;")
        self.btn_reset_map.clicked.connect(self.reset_map)
        self.sidebar.addWidget(self.btn_reset_map)

        self.sidebar.addStretch()
        
        main_layout.addLayout(self.sidebar)
        self.setLayout(main_layout)

        # Initialize background threads
        self.telem_thread = TelemetryReceiver()
        self.telem_thread.data_received.connect(self.update_telemetry)
        self.telem_thread.connection_status.connect(self.update_connection_ui)
        self.telem_thread.start()

        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_video)
        self.video_thread.ai_status_signal.connect(self.update_ai_button_status)
        self.video_thread.start()

        # Add LiDAR worker
        from workers import LidarReceiver
        self.lidar_thread = LidarReceiver()
        self.lidar_thread.data_received.connect(self.handle_lidar_data)
        self.lidar_thread.map_updated.connect(self.lidar_map.update_grid)
        self.lidar_thread.start()

        # Enable remote control
        self.control_thread = ControllerThread(ROBOT_IP)
        self.control_thread.status_signal.connect(self.update_controller_ui)

        # Local audio threads initialization
        self.audio_recv_thread = AudioReceiveThread(); self.audio_recv_thread.start()
        self.audio_send_thread = AudioSendThread(self.video_thread.robot_ip); self.audio_send_thread.start()

        # Timer Initialization
        self.record_timer = QTimer()
        self.record_timer.timeout.connect(self.update_recording_timer)
        self.record_start_time = 0

    def handle_lidar_data(self, ranges):
        # Passes the ranges and the AI human status to the map
        is_human = self.video_thread.is_person_in_view()
        self.lidar_map.set_human_status(is_human)

    def reset_map(self):
        # We need to wipe the occupancy grid for ALL particles
        for p in self.lidar_thread.pf.particles:
            p.og.occupancyGridVisited.fill(1)
            p.og.occupancyGridTotal.fill(2)
        
        # Force the widget to clear visually
        self.lidar_map.update_grid(None)
        print("RECON MAP RESET: Probabilistic grid cleared.")

    def restart_hardware_action(self):
        # Passes manual host IP to video thread and triggers robot
        self.video_thread.host_ip = self.ip_input.text()
        self.video_thread.start_remote_hardware()

    def toggle_mic(self):
        # Toggles microphone state and updates UI colors
        is_muted = "UNMUTED" in self.btn_mic.text()
        self.audio_send_thread.set_mute(is_muted)
        self.btn_mic.setText(f"MIC: {'MUTED' if is_muted else 'UNMUTED'}")
        self.btn_mic.setStyleSheet(f"background-color: #222; border: 1px solid {'#FF0000' if is_muted else '#00FF00'}; color: {'#FF0000' if is_muted else '#00FF00'}; padding: 8px;")

    def toggle_recording(self):
        # Toggles the recording state and updates the UI button styling
        if not self.video_thread.is_recording:
            # Check for or create a recordings directory
            rec_folder = "recordings"
            if not os.path.exists(rec_folder):
                os.makedirs(rec_folder)
                print(f"Created directory: {rec_folder}")

            now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            # Save the file into the new folder
            filename = os.path.join(rec_folder, f"quadconn_test_{now}.mp4")
            
            try:
                self.video_thread.start_recording(filename)
                
                # Check if it actually started before updating UI
                if self.video_thread.is_recording:
                    self.record_start_time = time.time()
                    self.record_timer.start(1000)
                    self.btn_record.setText("STOP RECORDING")
                    self.btn_record.setStyleSheet("background-color: #FF0000; color: white; padding: 10px; font-weight: bold;")
            except Exception as e:
                print(f"GUI Recording Error: {e}")

        else:
            # Stop logic
            self.video_thread.stop_recording()
            self.record_timer.stop()
            self.btn_record.setText("START RECORDING")
            self.btn_record.setStyleSheet("background-color: #222; border: 1px solid #FF0000; color: #FF0000; padding: 10px; font-weight: bold;")

    def update_recording_timer(self):
        # Calculate elapsed time and updates the button text
        elapsed = int(time.time() - self.record_start_time)
        mins, secs = divmod(elapsed, 60)
        self.btn_record.setText(f"STOP RECORDING ({mins:02}:{secs:02})")

    def update_speaker_volume(self, value):
        # Passes slider value to the local audio receiver
        self.audio_recv_thread.set_volume(value)

    def update_ai_button_status(self, status):
        # Updates UI styling based on AI model state
        if status == "LOADING":
            self.btn_ai.setText("PERSON DETECTION: INITIALIZING...")
            self.btn_ai.setStyleSheet("border: 1px solid #FFA500; color: #FFA500;")
        elif status == "READY":
            self.btn_ai.setText("PERSON DETECTION: ON")
            self.btn_ai.setStyleSheet("border: 1px solid #00FF00; color: #00FF00;")

    def toggle_ai(self):
        # Enables/Disables YOLO overlay in real-time
        if "INITIALIZING" in self.btn_ai.text(): return
        self.video_thread.ai_enabled = not self.video_thread.ai_enabled
        self.btn_ai.setText(f"PERSON DETECTION: {'ON' if self.video_thread.ai_enabled else 'OFF'}")
        self.btn_ai.setStyleSheet(f"border: 1px solid {'#00FF00' if self.video_thread.ai_enabled else '#FF0000'}; color: {'#00FF00' if self.video_thread.ai_enabled else '#FF0000'};")

    def toggle_controller(self):
        if not self.control_thread.isRunning():
            self.control_thread.start()
            self.btn_controller.setText("GAMEPAD: SEARCHING...")
        else:
            self.control_thread.stop()
            self.btn_controller.setText("ENABLE GAMEPAD")
            self.btn_controller.setStyleSheet("background-color: #222; color: #FFF;")

    def update_controller_ui(self, status):
        self.btn_controller.setText(f"GAMEPAD: {status}")
        if "CONNECTED" in status:
            self.btn_controller.setStyleSheet("background-color: #004400; color: #00FF00; border: 1px solid #00FF00;")
        else:
            self.btn_controller.setStyleSheet("background-color: #440000; color: #FF0000; border: 1px solid #FF0000;")

    def update_video(self, pixmap):
        # Renders latest processed frame to UI
        self.video_label.setPixmap(pixmap.scaled(
            self.video_label.size(), 
            Qt.AspectRatioMode.IgnoreAspectRatio, 
            Qt.TransformationMode.SmoothTransformation
        ))
        # Check the boolean state from workers.py
        is_human = self.video_thread.is_person_in_view()
        
        # Update the proximity box
        if is_human:
            self.lbl_ai_proximity.setText("HUMAN DETECTED")
            self.lbl_ai_proximity.setStyleSheet("background-color: #770000; border: 1px solid #FF0000; color: #FFFFFF; padding: 5px; font-weight: bold;")
        else:
            self.lbl_ai_proximity.setText("NO DETECTION")
            self.lbl_ai_proximity.setStyleSheet("background-color: #002200; border: 1px solid #00FF00; color: #00FF00; padding: 5px; font-weight: bold;")

    def update_connection_ui(self, is_online):
        # Updates status indicator based on telemetry reception
        self.lbl_status.setText("ONLINE" if is_online else "OFFLINE")
        self.lbl_status.setStyleSheet(f"color: {'#00FF00' if is_online else '#FF0000'}; border: 1px solid {'#00FF00' if is_online else '#FF0000'}; padding: 4px;")

    def update_telemetry(self, data):
        from PyQt6.QtWidgets import QMessageBox
        
        total_v = 0
        total_p = 0
        faults = []

        # Aggregate data
        for i in range(MOTOR_COUNT):
            m = data.motor_instance[i]
            total_v += m.voltage
            total_p += m.power
            if m.fault != 0:
                faults.append(f"M{i+1}")
            
            # Color individual motor boxes based on power/fault
            m_color = "#FF0000" if m.fault != 0 or m.power > (POWER_LIMIT_W/12) else "#00FF00"
            self.motor_labels[i].setStyleSheet(f"border: 1px solid {m_color}; color: {m_color}; font-size: 10px;")
            self.motor_labels[i].setText(f"M{i+1} | {m.position:.1f}\n{m.voltage:.1f}V | {m.power:.1f}W")

        avg_v = total_v / MOTOR_COUNT
        self.lbl_volt.setText(f"BATT: {avg_v:.1f} V")
        self.lbl_power.setText(f"PWR: {total_p:.1f} W")

        # Evaluate system health
        current_alert = ""
        is_critical = False

        if avg_v < VOLT_CRITICAL:
            current_alert = "CRITICAL BATTERY"
            is_critical = True
        elif faults:
            current_alert = f"MOTOR FAULT: {', '.join(faults)}"
            is_critical = True
        elif total_p > POWER_LIMIT_W:
            current_alert = "HIGH POWER DRAW WARNING"
        
        # Update visuals
        if is_critical:
            self.lbl_alert_banner.setText(f"{current_alert}")
            self.lbl_alert_banner.setStyleSheet("background-color: #770000; color: #FFFFFF; font-weight: bold; border: 2px solid #FF0000;")
            self.lbl_volt.setStyleSheet("color: #FF0000; font-size: 14px; font-weight: bold;")
            
            # Only pop up if it's been more than 30 seconds since the last one
            if time.time() - self.last_popup_time > 30:
                self.last_popup_time = time.time()
                QMessageBox.critical(self, "SYSTEM ALERT", f"CRITICAL FAILURE DETECTED:\n{current_alert}")
        
        elif current_alert: # Warning state
            self.lbl_alert_banner.setText(f"NOTICE: {current_alert}")
            self.lbl_alert_banner.setStyleSheet("background-color: #443300; color: #FFA500; font-weight: bold; border: 1px solid #FFA500;")
        
        else: # All systems green
            self.lbl_alert_banner.setText("SYSTEM HEALTHY")
            self.lbl_alert_banner.setStyleSheet("background-color: #002200; color: #00FF00; font-weight: bold; border: 1px solid #00FF00;")
            self.lbl_volt.setStyleSheet("color: #FFFFFF;")

    def closeEvent(self, event):
        # Ensures local and remote processes are killed when closing the window
        self.telem_thread.stop()
        self.video_thread.stop()
        self.lidar_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = QuadconnDashboard()
    gui.show()
    sys.exit(app.exec())