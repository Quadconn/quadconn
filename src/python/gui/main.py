### MAIN ###

import sys
import os
import datetime
import time
import matplotlib
matplotlib.use('Agg') # Force the "Silent" backend
import matplotlib.pyplot as plt
import numpy as np

# PyQt6 UI Components
from PyQt6.QtCore import Qt, QPoint, pyqtSignal, QTimer, QPointF
from PyQt6.QtWidgets import (QApplication, QLabel, QVBoxLayout, QHBoxLayout,
                             QWidget, QGridLayout, QPushButton, QLineEdit, QSlider)
from PyQt6.QtGui import QPainter, QImage, QPen, QColor, QBrush

# Custom Imports from your modular files
from configurations import *
from workers import VideoThread, TelemetryReceiver, AudioReceiveThread, AudioSendThread, ControllerThread
from motor_diagnostics import MOTOR_COUNT

class LidarMapWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.map_image = None
        self.robot_pos = (0, 0, 0)  # Stores current (x, y, theta) in feet
        
        # The coordinate boundaries for the current map view in feet.
        # This defines the "world" coordinates that map to our pixel space.
        self.map_limits = [-50, 50, -50, 50]  # [minX, maxX, minY, maxY]

        # Ensure the widget remains at a constant size within the GUI layout
        self.setFixedSize(350, 350) 
        
        # Dark theme styling to match the rest of the dashboard
        self.setStyleSheet("border: 1px solid #444; background-color: #111;")

        # Initialize the widget with a neutral grey 'blank' map (0.5 probability).
        # This prevents a black flicker or empty box while waiting for the first SLAM packet.
        blank = np.full((201, 201), 0.5)
        self.update_map(blank, 0, 0, 0, False)

    def update_map(self, array, x, y, theta, human_detected):
        """
        Receives processed data from the LidarSLAMWorker and prepares it for rendering.
        """
        # 1. Conversion: Maps occupancy probabilities (0.0 to 1.0) to grayscale pixel values (0 to 255).
        self._map_buffer = (array * 255).astype(np.uint8)  # one array, stored on self
        h, w = self._map_buffer.shape
        
        # 2. Memory Management: Wrap the NumPy buffer into a QImage for fast GPU rendering.
        # Using Format_Grayscale8 as it is the most memory-efficient for occupancy grids.
        self.map_image = QImage(self._map_buffer.data, w, h, w, QImage.Format.Format_Grayscale8)
        
        # 3. Synchronize the robot's estimated position and heading
        self.robot_pos = (x, y, theta)
        
        # 4. Notify the GUI that the widget needs to be redrawn (triggers paintEvent)
        self.update()

    def paintEvent(self, event):
        """
        The low-level drawing function. This handles the actual pixel mapping for the map and robot.
        """
        # Safety check: do not attempt to draw if a map hasn't been generated yet
        if not self.map_image:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Draw the grayscale occupancy grid stretched to fill the widget area
        target_rect = self.contentsRect()
        painter.drawImage(target_rect, self.map_image)

        # Coordinate Transformation: Convert 'Feet' coordinates to 'Pixel' coordinates
        view_half = 50.0
        min_x = self.robot_pos[0] - view_half
        max_x = self.robot_pos[0] + view_half
        min_y = self.robot_pos[1] - view_half
        max_y = self.robot_pos[1] + view_half
        
        # Calculate X pixel: (Relative position in feet / Total feet range) * Widget Width
        px = (self.robot_pos[0] - min_x) / (max_x - min_x) * self.width()
        
        # Calculate Y pixel: We invert the Y axis because pixel (0,0) is at the TOP-left.
        py = (1.0 - (self.robot_pos[1] - min_y) / (max_y - min_y)) * self.height()

        # Render the Robot Marker (Red Dot)
        painter.setBrush(QColor(255, 0, 0))
        painter.setPen(QPen(Qt.GlobalColor.white, 2))
        painter.drawEllipse(int(px) - 5, int(py) - 5, 10, 10)

        # Render the Heading Line (shows where the robot is facing)
        # line_len is in pixels; we use trig to find the tip of the line based on theta
        line_len = 15
        head_x = px + line_len * np.cos(self.robot_pos[2])
        head_y = py - line_len * np.sin(self.robot_pos[2])
        painter.drawLine(int(px), int(py), int(head_x), int(head_y))

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
        
        # Gamepad Monitor UI
        self.sidebar.addWidget(QLabel("GAMEPAD MONITOR:"))
        self.gp_monitor_layout = QGridLayout()
        self.gp_items = {}
        
        # 16 Buttons (4x4 Grid)
        btn_labels = [
            "A", "B", "X", "Y", 
            "LB", "RB", "LT", "RT", 
            "SEL", "STA", "L3", "R3", 
            "UP", "DN", "LF", "RI", 
            "HOM"
        ]
        
        for i, b in enumerate(btn_labels):
            lbl = QLabel(b)
            lbl.setFixedSize(35, 20)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("background-color: #222; color: #444; border-radius: 3px; font-size: 8px; font-weight: bold;")
            self.gp_monitor_layout.addWidget(lbl, i // 4, i % 4)
            self.gp_items[b] = lbl
        
        # Stick Status (4 Labels for LX, LY, RX, RY)
        self.stick_layout = QHBoxLayout()
        self.stick_displays = {}
        for axis in ["LX", "LY", "RX", "RY"]:
            lbl = QLabel(f"{axis}: 0.0")
            lbl.setStyleSheet("color: #00FF00; font-size: 9px; font-family: monospace;")
            self.stick_layout.addWidget(lbl)
            self.stick_displays[axis] = lbl
            
        self.sidebar.addLayout(self.gp_monitor_layout)
        self.sidebar.addLayout(self.stick_layout)
        
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
        # --- NEW SLAM INTEGRATION ---
        # 1. Import the new worker we just built
        from workers import LidarSLAMWorker
        
        # 2. Initialize the SLAM Engine
        self.lidar_thread = LidarSLAMWorker()
        
        # 3. Connect the Signal to the Widget
        # NOTE: We connect it to 'update_map', which is the method in your LidarMapWidget
        self.lidar_thread.map_updated.connect(self.lidar_map.update_map)
        
        # 4. Start the engine
        self.lidar_thread.start()

        # Enable remote control
        self.control_thread = ControllerThread(ROBOT_IP)
        self.control_thread.status_signal.connect(self.update_controller_ui)
        self.control_thread.controller_state.connect(self.update_gamepad_monitor)

        # Local audio threads initialization
        self.audio_recv_thread = AudioReceiveThread(); self.audio_recv_thread.start()
        self.audio_send_thread = AudioSendThread(self.video_thread.robot_ip); self.audio_send_thread.start()

        # Timer Initialization
        self.record_timer = QTimer()
        self.record_timer.timeout.connect(self.update_recording_timer)
        self.record_start_time = 0

    def reset_map(self):
        # New Particle-based reset
        if hasattr(self, 'lidar_thread') and self.lidar_thread.pf:
            for p in self.lidar_thread.pf.particles:
                p.og.occupancyGridVisited.fill(1)
                p.og.occupancyGridTotal.fill(2)
            
            # Create a "neutral gray" blank map (201x201 based on your 100ft/0.5 resolution)
            blank_map = np.full((201, 201), 0.5) 
            self.lidar_map.update_map(blank_map, 0, 0, 0, False)
            print("RECON MAP RESET: SLAM Particle grids cleared.")
        else:
            print("Reset failed: SLAM engine not initialized yet.")

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
    # Handles the activation and deactivation of the gamepad control subsystem.
    # This function manages the lifecycle of the ControllerThread and updates the UI accordingly.
    
        # Safety check: Verify that the control_thread object was successfully created in __init__
        if not hasattr(self, 'control_thread') or self.control_thread is None:
            print("Error: Controller thread not initialized in __init__")
            return

        # Phase 1: Logic for stopping an active controller session
        if self.control_thread.isRunning():
            print("Stopping existing controller thread...")
        
            # Signals the background thread to exit its loop and wait for it to finish
            self.control_thread.stop()
        
            # Reset the button appearance to indicate the system is currently idle
            self.btn_controller.setText("ENABLE GAMEPAD")
            self.btn_controller.setStyleSheet("background-color: #222; color: #FFF;")
        
            # Reset the Gamepad monitor visual indicators by passing an empty state
            # This prevents the UI from showing 'stuck' stick values after the thread stops
            self.update_gamepad_monitor({}) 
        
        else:
            # Phase 2: Logic for starting a fresh controller session
            print("Starting new controller thread...")
        
            # Reset velocity and orientation variables to zero before starting
            # This prevents 'ghost movement' where the robot moves based on old data
            self._robot_speed_mps = 0.0
            self._robot_theta_rad = 0.0
        
            # Begin the background SDL/Pygame event loop in the worker thread
            self.control_thread.start()
        
            # Update UI to provide immediate feedback that control is now active
            self.btn_controller.setText("GAMEPAD: ON")
            self.btn_controller.setStyleSheet("background-color: #2e7d32; color: white;")

    def update_controller_ui(self, status):
        self.btn_controller.setText(f"GAMEPAD: {status}")
        if "CONNECTED" in status:
            self.btn_controller.setStyleSheet("background-color: #004400; color: #00FF00; border: 1px solid #00FF00;")
        else:
            self.btn_controller.setStyleSheet("background-color: #440000; color: #FF0000; border: 1px solid #FF0000;")

    def update_gamepad_monitor(self, state):
    # Slot triggered by the ControllerThread's 'controller_state' signal.
    # This handles both the visual feedback on the UI and passes physics data to the SLAM engine.
    
    # Helper function to change UI element colors based on button press status
        def set_active(label_key, is_pressed):
            if label_key in self.gp_items:
                # Active buttons turn bright green; inactive buttons stay dark grey
                color = "#00FF00" if is_pressed else "#444"
                bg = "#004400" if is_pressed else "#222"
                self.gp_items[label_key].setStyleSheet(
                f"background-color: {bg}; color: {color}; border-radius: 3px; font-size: 8px; font-weight: bold;"
                )

        # Safety check: exit if for some reason the state dictionary is empty
        if not state:
            return

        # Update digital button displays using the raw button indices from the controller thread
        set_active("A", state.get("A"))
        set_active("B", state.get("B"))
        set_active("X", state.get("X"))
        set_active("Y", state.get("Y"))
        set_active("LB", state.get("LB"))
        set_active("RB", state.get("RB"))
        set_active("SEL", state.get("Select"))
        set_active("STA", state.get("Start"))
        set_active("L3", state.get("L3"))
        set_active("R3", state.get("R3"))
        set_active("HOM", state.get("Home"))
    
        # Update D-Pad (Directional Pad) visual indicators
        set_active("UP", state.get("UP"))
        set_active("DN", state.get("DOWN"))
        set_active("LF", state.get("LEFT"))
        set_active("RI", state.get("RIGHT"))

        # Analog Triggers: Visually light up if pulled more than 10% to account for slight hardware drift
        set_active("LT", state.get("LT", 0) > 0.1)
        set_active("RT", state.get("RT", 0) > 0.1)

        # Update text displays for the X and Y axes of both analog sticks
        self.stick_displays["LX"].setText(f"LX:{state.get('lx', 0):.1f}")
        self.stick_displays["LY"].setText(f"LY:{state.get('ly', 0):.1f}")
        self.stick_displays["RX"].setText(f"RX:{state.get('rx', 0):.1f}")
        self.stick_displays["RY"].setText(f"RY:{state.get('ry', 0):.1f}")

        # Integration step: Passing control data to the SLAM engine for dead reckoning
        # We use 'vx' directly for forward/backward velocity. This is signed,
        # meaning positive values move the robot forward and negative values move it backward.
        vx = state.get("vx", 0.0)
        vy = state.get("vy", 0.0)
        # Commanded yaw rate defines how fast the robot's heading is changing (radians/second)
        commanded_yaw = state.get("yaw_rate", 0.0)

        # If the SLAM thread is running, push the latest velocity and detection data to it
        if hasattr(self, 'lidar_thread') and self.lidar_thread.isRunning():
            # This method call serves as the thread-safe 'mailbox' between the UI and SLAM engine
            # We also pass the human detection status from the YOLO video thread
            self.lidar_thread.update_robot_state(
                speed_mps=vx,
                strafe_mps=vy,
                yaw_rate=commanded_yaw, 
                human_present=self.video_thread.is_person_in_view()
            )

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

        # 1. Extract the speed and heading from your telemetry data object.
        # Replace 'forward_velocity' and 'yaw' with the actual names used in your C++ struct.
        speed = getattr(data, 'forward_velocity', 0.0)
        heading = getattr(data, 'yaw', 0.0)

        # 2. Feed it into the SLAM engine to help it predict the robot's next position.
        if hasattr(self, 'lidar_thread') and self.lidar_thread is not None:
            self.lidar_thread.update_motion(speed, heading)

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