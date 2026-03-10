import os
import sys

# This block ensures the Torch engine loads its memory space and DLLs before GStreamer or PyQT6 to prevent initialization routine failure error (WinError 1114)

os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
os.environ["OMP_NUM_THREADS"] = "1"

try:
    # Manually pointing Windows to torch lib folder before app starts
    venv_base = os.path.dirname(sys.executable)
    torch_lib = os.path.join(venv_base, "Lib", "site-packages", "torch", "lib")
    
    # Directly load the venv's DLL directory into the OS search path
    if os.path.exists(torch_lib):
        os.add_dll_directory(torch_lib)
   
    # Pre-loading Torch at global level locks the libraries into memory
    import torch
    torch.set_num_threads(1)
    print("AI: Global Torch Engine Initialized Successfully")
except Exception as e:
    # Safe to ignore on macOS or environments without Torch installed
    print(f"AI: Global Setup Note: {e}")

## Standard Imports ##
import socket
import subprocess
import numpy as np
import cv2
import traceback
from ctypes import sizeof, memmove, addressof

# GStreamer Integration: Handles raw H.264 stream decoding
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# PyQt6 UI Components
from PyQt6.QtCore import QThread, pyqtSignal, Qt, QMutex
from PyQt6.QtWidgets import (QApplication, QLabel, QVBoxLayout, QHBoxLayout,
                             QWidget, QGridLayout, QPushButton, QLineEdit)
from PyQt6.QtGui import QImage, QPixmap

# C-compatible structure for 1,152-byte motor packets
from motor_diagnostics import MotorDiagnosticsArray, MotorInfo, MOTOR_COUNT

# Initialize GStreamer core
Gst.init(None)

class TelemetryReceiver(QThread):

    # Handles background UDP reception of motor diagnostic data on Port 808 by directly mapping raw network bytes into C-style memory structures
    data_received = pyqtSignal(object)
    connection_status = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.running = True
        self.daemon = True # Ensures thread dies if main application closes

    def run(self):
        # Setup UDP socket with a 1-second timeout for 'Offline' detection
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0)
        try:
            sock.bind(("0.0.0.0", 808))
        except Exception as e:
            print(f"Socket Bind Error: {e}")
       
        expected_size = sizeof(MotorDiagnosticsArray)
        print("Telemetry Receiver Active (Port 808)")

        while self.running:
            try:
                # Receive raw bytes and map them directly into the C-style structure
                data, addr = sock.recvfrom(4096)
                if len(data) == expected_size:
                    telem_array = MotorDiagnosticsArray()
                    memmove(addressof(telem_array), data, expected_size)
                    self.data_received.emit(telem_array)
                    self.connection_status.emit(True)
            except socket.timeout:
                self.connection_status.emit(False)
            except Exception:
                self.connection_status.emit(False)
        sock.close()

    def stop(self):
        # Terminates socket loop safely
        self.running = False
        self.quit()

class InferenceWorker(QThread):

    # Ensures YOLOv8 inference in a parallel thread while ensuring that AI processing does not slow down the 30 FPS GStreamer video feed
    results_ready = pyqtSignal(object)
    model_loaded = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.model = None
        self.frame_to_process = None
        self.running = True
        self.mutex = QMutex() # Prevents race conditions during frame handovers
        self.daemon = True

    def run(self):
        # Since torch is already loaded globally, we just load the YOLO weights
        try:
            from ultralytics import YOLO
            print("AI: Loading YOLOv8 weights...")
            self.model = YOLO('yolov8n.pt')
           
            # Final run to ensure the thread has access to the global engine
            dummy_frame = np.zeros((160, 160, 3), dtype=np.uint8)
            self.model.predict(dummy_frame, device='cpu', verbose=False)
           
            print("AI: Inference Worker Ready!")
            self.model_loaded.emit()
        except Exception as e:
            print(f"InferenceWorker Initialization Error: {e}")
            traceback.print_exc()
            return
       
        while self.running:
            frame = None
            self.mutex.lock()
            if self.frame_to_process is not None:
                frame = self.frame_to_process.copy()
                self.frame_to_process = None
            self.mutex.unlock()

            if frame is not None and self.model:
                # Detect people only (class 0) with a 50% confidence threshold
                results = self.model.predict(frame, classes=[0], conf=0.5, device='cpu', verbose=False)
                if len(results) > 0:
                    self.results_ready.emit(results[0].plot())
            else:
                self.msleep(30) # Optimized polling rate for CPU stability

    def update_frame(self, frame):
        # Submits a new frame for processing if the worker is ready
        if self.mutex.tryLock():
            self.frame_to_process = frame
            self.mutex.unlock()

class VideoThread(QThread):
    # Manages GStreamer pipeline and handles remote SSH triggers for robot hardware
    change_pixmap_signal = pyqtSignal(QPixmap)
    ai_status_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        # Robot Configuration
        self.robot_ip = "100.81.189.79"
        self.host_ip = "" # Your Tailscale IP
        self.robot_exec = "/home/quadconn/gui_branch/quadconn/build/camera_stream"

        self.ai_enabled = True
        self.pipeline = None
        self.loop = None
        self.daemon = True
       
        # Initialize parallel AI worker
        self.ai_worker = InferenceWorker()
        self.ai_worker.results_ready.connect(self.on_ai_results)
        self.ai_worker.model_loaded.connect(lambda: self.ai_status_signal.emit("READY"))
        self.latest_ai_plot = None

    def run(self):
        # Start AI worker and GStreamer pipeline
        self.ai_status_signal.emit("LOADING")
        self.ai_worker.start()

        # GStreamer Launch String: Receives H.264 via UDP on Port 5000
        gst_str = (
            "udpsrc port=5000 ! "
            "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264,payload=(int)96 ! "
            "rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            "video/x-raw,format=BGR ! appsink name=sink emit-signals=True sync=false"
        )
        self.pipeline = Gst.parse_launch(gst_str)
        sink = self.pipeline.get_by_name("sink")
        sink.connect("new-sample", self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)

        # Main loop to keep GStreamer signals alive
        self.loop = GLib.MainLoop()
        self.loop.run()

    def start_remote_hardware(self):
        # Triggers the robot's camera binary via SSH with port cleanup
        if not self.host_ip: return
        print(f"Connecting to robot at {self.robot_ip}...")
        
        # Cleanup: Force kill any lingering camera processes and free the video device
        cleanup = f"ssh -o ConnectTimeout=3 quadconn@{self.robot_ip} \"sudo pkill -9 camera_stream; sudo fuser -k /dev/video0\""
        subprocess.run(cleanup, shell=True)
        
        # Launch: Execute binary on robot pointing back to this computer
        launch = f"ssh -o ConnectTimeout=3 quadconn@{self.robot_ip} \"{self.robot_exec} {self.host_ip}\""
        subprocess.Popen(launch, shell=True)

    def on_ai_results(self, plotted_frame):
        # Stores the latest detection overlay for persistent rendering
        self.latest_ai_plot = plotted_frame

    def on_new_sample(self, sink):
        # Processes raw GStreamer buffers into PyQt6-compatible images
        sample = sink.emit("pull-sample")
        if not sample: return Gst.FlowReturn.ERROR
        buf = sample.get_buffer()
        caps = sample.get_caps()
        res, map_info = buf.map(Gst.MapFlags.READ)
        
        if res:
            width = caps.get_structure(0).get_value("width")
            height = caps.get_structure(0).get_value("height")
            frame = np.frombuffer(map_info.data, dtype=np.uint8).reshape((height, width, 3))
            
            # Submit frame to parallel AI thread
            if self.ai_enabled: 
                self.ai_worker.update_frame(frame)
            
            # Decide whether to show raw video or AI annotated frame
            if self.ai_enabled and self.latest_ai_plot is not None:
                display_frame = cv2.resize(self.latest_ai_plot, (width, height))
            else:
                display_frame = frame
            
            # Color conversion (BGR to RGB) and signal emission to the main GUI thread
            rgb_image = cv2.cvtColor(display_frame, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            qimg = QImage(rgb_image.data, w, h, ch * w, QImage.Format.Format_RGB888)
            self.change_pixmap_signal.emit(QPixmap.fromImage(qimg))
            buf.unmap(map_info)
        return Gst.FlowReturn.OK

    def stop(self):
        # Safe shutdown of local threads and remote robot binaries
        if self.loop: self.loop.quit()
        if self.pipeline: self.pipeline.set_state(Gst.State.NULL)
        self.ai_worker.running = False
        subprocess.run(f"ssh -o ConnectTimeout=2 quadconn@{self.robot_ip} \"sudo pkill -9 camera_stream\"", shell=True)
        self.quit()

## Main User Interface ##
class QuadconnDashboard(QWidget):
    # Integrates video stream, manual network configuration, and 12-motor telemetry grid
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quadconn Senior Design Dashboard")
        self.setFixedSize(1200, 750)
        self.setStyleSheet("background-color: #0A0A0A; color: #FFFFFF;")
        
        # Main Layout
        main_layout = QHBoxLayout()
        
        # Live Video Display Label
        self.video_label = QLabel("PASTE YOUR TAILSCALE IP & START CAMERA")
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
        
        self.btn_start = QPushButton("SET IP & START CAMERA")
        self.btn_start.setStyleSheet("background-color: #004400; color: white; padding: 10px; font-weight: bold;")
        self.btn_start.clicked.connect(self.restart_camera_action)
        self.sidebar.addWidget(self.btn_start)
        
        # Status Header
        self.status_header = QHBoxLayout()
        self.lbl_volt = QLabel("BATTERY: -- V")
        self.lbl_volt.setStyleSheet("font-size: 20px; color: #00FF00; font-weight: bold;")
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

    def restart_camera_action(self):
        # Passes manual host IP to video thread and triggers robot
        self.video_thread.host_ip = self.ip_input.text()
        self.video_thread.start_remote_hardware()

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
            
            # Highlight faults in RED, healthy active motors in GREEN
            color = "#00FF00" if m.fault == 0 else "#FF0000"
            self.motor_labels[i].setStyleSheet(f"border: 2px solid {color}; color: {color}; font-size: 10px; font-weight: bold;")
        
        # Display average battery health in the header
        self.lbl_volt.setText(f"BATTERY: {total_v/MOTOR_COUNT:.1f} V")

    def closeEvent(self, event):
        # Ensures robot processes are killed when closing the window
        self.telem_thread.stop()
        self.video_thread.stop()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = QuadconnDashboard()
    gui.show()
    sys.exit(app.exec())