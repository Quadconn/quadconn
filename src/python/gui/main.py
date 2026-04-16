### MAIN ###

import sys
import os
import datetime
import time

# PyQt6 UI Components
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtWidgets import (QApplication, QLabel, QVBoxLayout, QHBoxLayout,
                             QWidget, QGridLayout, QPushButton, QLineEdit, QSlider)

# Custom Imports from your modular files
from configurations import *
from workers import VideoThread, TelemetryReceiver, AudioReceiveThread, AudioSendThread
from motor_diagnostics import MOTOR_COUNT

## Main User Interface ##
class QuadconnDashboard(QWidget):
    # Integrates video stream, manual network configuration, 12-motor telemetry grid, and two-way audio
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quadconn Senior Design Dashboard")
        self.setFixedSize(1200, 750)
        self.setStyleSheet("background-color: #0A0A0A; color: #FFFFFF;")
        
        # Main Layout
        main_layout = QHBoxLayout()
        
        # Live Video Display Label
        self.video_label = QLabel("PASTE YOUR TAILSCALE IP & START SYSTEM")
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setFixedSize(780, 580)
        self.video_label.setStyleSheet("border: 2px solid #333; background-color: black;")
        main_layout.addWidget(self.video_label)
        
        # Sidebar Container
        self.sidebar = QVBoxLayout()
        
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
        
        # Status Header
        self.status_header = QHBoxLayout()
        self.lbl_volt = QLabel("BATTERY: -- V")
        self.lbl_status = QLabel("OFFLINE")
        self.lbl_status.setStyleSheet("font-size: 14px; color: #FF0000; font-weight: bold; border: 1px solid #FF0000; padding: 4px;")
        self.status_header.addWidget(self.lbl_volt)
        self.status_header.addWidget(self.lbl_status)
        self.sidebar.addLayout(self.status_header)

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

        # Record Function
        self.btn_record = QPushButton("START RECORDING")
        self.btn_record.setStyleSheet("background-color: #222; border: 1px solid #FF0000; color: #FF0000; padding: 10px; font-weight: bold;")
        self.btn_record.clicked.connect(self.toggle_recording)
        self.sidebar.addWidget(self.btn_record)
        
        # 12-Motor Grid
        self.grid = QGridLayout()
        self.motor_labels = []
        for i in range(MOTOR_COUNT):
            lbl = QLabel(f"M{i+1}\nWAITING...")
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("border: 1px solid #444; font-size: 10px; background-color: #111;")
            self.grid.addWidget(lbl, i // 3, i % 3)
            self.motor_labels.append(lbl)
        self.sidebar.addLayout(self.grid)
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

        # Local audio threads initialization
        self.audio_recv_thread = AudioReceiveThread(); self.audio_recv_thread.start()
        self.audio_send_thread = AudioSendThread(self.video_thread.robot_ip); self.audio_send_thread.start()

        # Timer Initialization
        self.record_timer = QTimer()
        self.record_timer.timeout.connect(self.update_recording_timer)
        self.record_start_time = 0

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

    def update_video(self, pixmap):
        # Renders latest processed frame to UI
        self.video_label.setPixmap(pixmap.scaled(self.video_label.size(), Qt.AspectRatioMode.KeepAspectRatio))

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
        # Calculates battery health and updates motor grid with fault highlights
        total_v = 0
        for i in range(MOTOR_COUNT):
            m = data.motor_instance[i]
            total_v += m.voltage
            self.motor_labels[i].setText(f"M{i+1} | {m.position:.2f}\n{m.voltage:.1f}V")
            color = "#00FF00" if m.fault == 0 else "#FF0000"
            self.motor_labels[i].setStyleSheet(f"border: 2px solid {color}; color: {color}; font-size: 10px; font-weight: bold;")
        self.lbl_volt.setText(f"BATTERY: {total_v/MOTOR_COUNT:.1f} V")

    def closeEvent(self, event):
        # Ensures local and remote processes are killed when closing the window
        self.telem_thread.stop()
        self.video_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = QuadconnDashboard()
    gui.show()
    sys.exit(app.exec())