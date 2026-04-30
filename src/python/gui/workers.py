### WORKERS ###

import json
import socket
import subprocess
import numpy as np
import cv2
import time
import threading
import math
import os
import pygame
import matplotlib.pyplot as plt
import ctypes
import iceoryx2 as iox2
plt.pause = lambda x: None 
plt.show = lambda: None
from ctypes import sizeof, memmove, addressof
from PyQt6.QtCore import QThread, pyqtSignal, QMutex
from PyQt6.QtGui import QImage, QPixmap

# GStreamer Integration: Handles raw H.264 stream decoding
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

from configurations import *
from motor_diagnostics import MotorDiagnosticsArray

# Initialize GStreamer core
Gst.init(None)

class AudioReceiveThread(QThread):
    # Handles background reception of robot mic audio on Port 3004 and plays it locally on the Mac
    def __init__(self):
        super().__init__()
        self.pipeline = None
        self.volume_element = None
        self.daemon = True

    def run(self):
        # Uses a 'tee' to play audio locally while simultaneously forwarding raw packets to an internal port for recording without causing latency
        gst_str = (
            f"udpsrc port={MIC_TO_SPEAKER_PORT} ! "
            "application/x-rtp,media=audio,clock-rate=48000,encoding-name=OPUS,payload=96 ! "
            "tee name=at "
            "at. ! queue ! rtpjitterbuffer latency=100 ! rtpopusdepay ! opusdec ! audioconvert ! audioresample ! "
            "volume name=vol_control volume=1.0 ! autoaudiosink sync=false "
            f"at. ! queue ! udpsink host=127.0.0.1 port={AUDIO_RECORD_PORT} sync=false"
        )
        self.pipeline = Gst.parse_launch(gst_str)
        self.volume_element = self.pipeline.get_by_name("vol_control")
        self.pipeline.set_state(Gst.State.PLAYING)
        
        self.loop = GLib.MainLoop()
        self.loop.run()

    def set_volume(self, value):
        # Maps slider value to GStreamer volume property (0.0 to 1.0)
        if self.volume_element:
            self.volume_element.set_property("volume", value / 100.0)

    def stop(self):
        # Safely stops the GLib loop and releases the GStreamer hardware
        if self.loop: self.loop.quit()
        if self.pipeline: self.pipeline.set_state(Gst.State.NULL)

class AudioSendThread(QThread):
    def __init__(self, robot_ip):
        super().__init__()
        self.robot_ip = robot_ip
        self.pipeline = None
        self.daemon = True

    def run(self):
        # Added 'queue' for buffering and 'sync=false' so the Mac doesn't wait for a global clock
        gst_str = (
            "osxaudiosrc ! queue ! audioconvert ! audioresample ! "
            "opusenc bitrate=64000 ! rtpopuspay ! "
            f"udpsink host={self.robot_ip} port={SPEAKER_TO_MIC_PORT} sync=false"
        )
        self.pipeline = Gst.parse_launch(gst_str)
        
        # Start in PAUSED to prepare the hardware without sending data yet
        self.pipeline.set_state(Gst.State.PAUSED)
        
        self.loop = GLib.MainLoop()
        self.loop.run()

    def set_mute(self, is_muted):
        if self.pipeline:
            # Toggling between PAUSED and PLAYING is much safer on macOS than NULL
            state = Gst.State.PAUSED if is_muted else Gst.State.PLAYING
            self.pipeline.set_state(state)
            print(f"AUDIO DEBUG: Mac Microphone state -> {state}")

    def stop(self):
        if self.loop: self.loop.quit()
        if self.pipeline: self.pipeline.set_state(Gst.State.NULL)

# Import files for particle filtering and grid management
from FastSlam import ParticleFilter
from OccupancyGrid import OccupancyGrid

class LidarSLAMWorker(QThread):
    # Signal emitted to main.py to update the GUI with the processed map and robot coordinates
    map_updated = pyqtSignal(np.ndarray, float, float, float)

    def __init__(self):
        super().__init__()
        self.running = True
        self.port = 6000 # UDP port for incoming LiDAR/Telemetry data
        
        # Thread lock ensures safe data exchange between the GUI and this background thread
        self._state_lock = threading.Lock()
        
        # Internal state variables updated by the controller
        self._robot_speed_mps = 0.0     # Signed linear velocity from sticks (m/s)
        self._robot_yaw_rate = 0.0      # Angular velocity from sticks (rad/s)
        self._human_present = False     # YOLO detection flag
        
        # The integrated heading tracks the absolute orientation of the robot over time.
        # It is updated by integrating the yaw rate (angular velocity * time delta).
        self.integrated_theta = 0.0 
        
        # Initialize SLAM and Grid parameters
        self.init_params()

    def init_params(self):
        # Occupancy Grid Parameters: [Width, Height, Start_Pose, Resolution, FOV, etc.]
        # Resolution is 0.2 feet per cell (approx 2.4 inches) for high-detail mapping.
        self.og_params = [150, 150, None, 0.2, math.radians(120), 164.042, 300, 1.0]
        
        # Scan Matching Parameters: [Search Radius (ft), Sigma, etc.]
        # Search radius of 3.0ft allows for robust matching at normal walking speeds.
        self.sm_params = [3.0, 0.1, 4, 0.3, 0.5, 0.2, 0.15, 5]
        
        # The Particle Filter engine will be initialized upon receipt of the first data packet
        self.pf = None

    def run(self):
        # Configure UDP socket to listen for robot telemetry and LiDAR packets
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', self.port))
        sock.settimeout(1.0)
        
        count = 0
        last_scan_time = None
        
        print(f"SLAM Engine: Listening on Port {self.port}...")

        while self.running:
            try:
                # Receive raw scan data (JSON format) from the robot
                data, _ = sock.recvfrom(65507)
                scan_entry = json.loads(data.decode('utf-8'))
                
                # Default safety values for coordinates if not provided in the packet
                scan_entry.setdefault('x', 0.0)
                scan_entry.setdefault('y', 0.0)
                scan_entry.setdefault('theta', 0.0)

                # Initialize the Particle Filter only once using the first packet's coordinates
                if self.pf is None:
                    self.og_params[2] = scan_entry 
                    self.pf = ParticleFilter(8, self.og_params, self.sm_params)
                    
                    # Sync the initial heading with the robot's starting orientation
                    self.integrated_theta = scan_entry['theta']
                    print("SLAM: Particle Filter Initialized.")

                # Calculate the time delta (dt) between packets for accurate dead reckoning
                count += 1
                now = time.monotonic()
                dt = (now - last_scan_time) if last_scan_time is not None else 0.0
                last_scan_time = now

                # Safely pull the latest stick inputs from the thread mailbox
                with self._state_lock:
                    # Convert m/s from controller to feet/s for the SLAM coordinate system
                    speed_fps = self._robot_speed_mps * 3.28084 
                    current_yaw_rate = self._robot_yaw_rate

                # Update absolute heading using Euler integration
                # New Heading = Current Heading + (Rotation Rate * Time Elapsed)
                self.integrated_theta += (current_yaw_rate * dt)

                # Determine if the robot is currently in motion (linear or angular)
                isMoving = abs(speed_fps) > 1e-3 or abs(current_yaw_rate) > 1e-3
                
                # Construct motion dictionary to guide the Particle Filter's prediction step
                motion = {
                    'speed': speed_fps, 
                    'orientation': self.integrated_theta, # Pass absolute heading
                    'dt': dt
                } if isMoving else None

                # Core SLAM Step 1: Update particles based on motion and LiDAR scan matching
                self.pf.updateParticles(scan_entry, count, motion)
                
                # Core SLAM Step 2: Resample particles if weights become unbalanced
                if self.pf.weightUnbalanced():
                    self.pf.resample()

                # Optimization: Update the GUI map every 3 frames to save CPU cycles
                if count % 3 == 0:
                    # Select the particle with the highest confidence weight
                    best = max(self.pf.particles, key=lambda p: p.weight)
                    curr_x = best.xTrajectory[-1]
                    curr_y = best.yTrajectory[-1]
                    curr_theta = self.integrated_theta 

                    # Map Slicing: Extract a 100ft x 100ft window centered on the robot
                    og = best.og
                    view_half = 50.0 
                    x_range = [curr_x - view_half, curr_x + view_half]
                    y_range = [curr_y - view_half, curr_y + view_half]

                    # Clamp the window within the actual occupancy grid boundaries
                    x_range = [max(x_range[0], og.mapXLim[0]), min(x_range[1], og.mapXLim[1])]
                    y_range = [max(y_range[0], og.mapYLim[0]), min(y_range[1], og.mapYLim[1])]

                    # Convert world feet coordinates to grid indices
                    x_idx, y_idx = og.convertRealXYToMapIdx(x_range, y_range)

                    # Extract the sub-section of the occupancy grid for visualization
                    visited = og.occupancyGridVisited[y_idx[0]:y_idx[1], x_idx[0]:x_idx[1]]
                    total   = og.occupancyGridTotal  [y_idx[0]:y_idx[1], x_idx[0]:x_idx[1]]

                    # Calculate occupancy ratio (0.5 represents unknown territory)
                    with np.errstate(divide='ignore', invalid='ignore'):
                        ratio = np.where(total > 0, visited / total, 0.5)
                    
                    # Flip the grid for correct vertical orientation in the GUI display
                    ogMap = np.flipud(1.0 - ratio)
                    
                    # Send the processed map and robot pose back to the main GUI thread
                    self.map_updated.emit(ogMap, curr_x, curr_y, curr_theta)

            except socket.timeout:
                continue
            except Exception as e:
                print(f"SLAM Error: {e}")

    def update_robot_state(self, speed_mps, yaw_rate, human_present):
        # Thread-safe method called by main.py to pass controller data to the SLAM engine.
        # Acts as the 'Mailbox' for cross-thread communication.
        with self._state_lock:
            self._robot_speed_mps = float(speed_mps)
            self._robot_yaw_rate = float(yaw_rate)
            self._human_present = bool(human_present)

    def stop(self):
        # Safely shut down the worker thread
        self.running = False
        self.wait()

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
            sock.bind(("0.0.0.0", TELEMETRY_PORT))
        except Exception as e:
            print(f"Socket Bind Error: {e}")
       
        expected_size = sizeof(MotorDiagnosticsArray)
        print(f"Telemetry Receiver Active {TELEMETRY_PORT}")

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
    human_status_signal = pyqtSignal(bool)
    model_loaded = pyqtSignal()

    def __init__(self):
        super().__init__()
        self.model = None
        self.frame_to_process = None
        self.running = True
        self.mutex = QMutex()
        self.daemon = True
        self.human_detected = False

        # iceoryx2 publisher for human detection status
        self.iox_node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        self.iox_service = (
            self.iox_node
            .service_builder(iox2.ServiceName.new("HumanDetection"))
            .publish_subscribe(ctypes.c_bool)
            .open_or_create()
        )
        self.iox_publisher = self.iox_service.publisher_builder().create()

    def get_human_status(self):
        return self.human_detected

    def run(self):
        # Since torch is already loaded globally, we just load the YOLO weights
        try:
            from ultralytics import YOLO
            print("Loading YOLOv8 weights...")
            self.model = YOLO(YOLO_WEIGHTS)
           
            # Final run to ensure the thread has access to the global engine
            dummy_frame = np.zeros((160, 160, 3), dtype=np.uint8)
            self.model.predict(dummy_frame, device='cpu', verbose=False)
           
            print("Person Detection Ready!")
            self.model_loaded.emit()
        except Exception as e:
            print(f"InferenceWorker Initialization Error: {e}")
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

                # Checks if the results list is not empty AND if there are bounding boxes found
                is_person_present = len(results) > 0 and len(results[0].boxes) > 0
                
                # Only emit a signal if the status actually changed
                if is_person_present != self.human_detected:
                    self.human_detected = is_person_present
                    self.human_status_signal.emit(self.human_detected)
                    # Publishing value to Lidar Plotter if human is detected
                    sample = self.iox_publisher.loan_uninit()
                    sample = sample.write_payload(ctypes.c_bool(is_person_present))
                    sample.send()
                    
                if is_person_present:
                    self.results_ready.emit(results[0].plot())
                else:
                    # Tell the UI to clear the previous detection overlay
                    self.results_ready.emit(None) 
            else:
                self.msleep(30)

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
        self.robot_ip = ROBOT_IP
        self.host_ip = "" # Your Tailscale IP
        self.robot_exec = ROBOT_EXEC
        self.record_pipeline = None

        self.ai_enabled = True
        self.pipeline = None
        self.loop = None
        self.daemon = True
       
        # Initialize parallel AI worker
        self.ai_worker = InferenceWorker()
        self.ai_worker.results_ready.connect(self.on_ai_results)
        self.ai_worker.model_loaded.connect(lambda: self.ai_status_signal.emit("READY"))
        self.latest_ai_plot = None

        # Recording State
        self.is_recording = False
        self.record_mutex = QMutex()

    def run(self):
        # Start AI worker and GStreamer pipeline
        self.ai_status_signal.emit("LOADING")
        self.ai_worker.start()

        # Uses a 'tee' to split the stream for live AI dashboard processing and raw packet forwarding to the internal recording port
        gst_str = (
            f"udpsrc port={VIDEO_PORT} ! "
            "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! "
            "tee name=t "
            "t. ! queue ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink emit-signals=True sync=false "
            f"t. ! queue ! udpsink host=127.0.0.1 port={VIDEO_RECORD_PORT} sync=false"
        )
        self.pipeline = Gst.parse_launch(gst_str)
        sink = self.pipeline.get_by_name("sink")
        sink.connect("new-sample", self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)

        # Main loop to keep GStreamer signals alive
        self.loop = GLib.MainLoop()
        self.loop.run()

    def start_recording(self, filename):
        # Initializes the GStreamer recording pipeline
        self.record_mutex.lock()
        try:
            # Launches a recording pipeline with a full RTP handshake and timestamp resetting to ensure immediate, synchronized MP4 file generation
            gst_record_str = (
                f"mp4mux name=mux ! filesink location={filename} "
                f"udpsrc port={VIDEO_RECORD_PORT} timeout=1000000000 do-timestamp=true ! "
                f"application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! "
                f"rtph264depay ! h264parse config-interval=-1 ! queue max-size-buffers=1000 ! mux.video_0 "
                f"udpsrc port={AUDIO_RECORD_PORT} timeout=1000000000 do-timestamp=true ! "
                f"application/x-rtp,media=audio,clock-rate=48000,encoding-name=OPUS,payload=96 ! "
                f"rtpopusdepay ! opusdec ! audioconvert ! audioresample ! avenc_aac ! queue max-size-buffers=1000 ! mux.audio_0"
            )
            
            self.record_pipeline = Gst.parse_launch(gst_record_str)
            self.record_pipeline.set_state(Gst.State.PLAYING)
            self.is_recording = True
            print(f"Recording started: {filename}")
            
        except Exception as e:
            print(f"FAILED TO START RECORDING: {e}")
            self.is_recording = False
        finally:
            self.record_mutex.unlock()

    def stop_recording(self):
        # Safely stops the recording pipeline without freezing the dashboard
        self.record_mutex.lock()
        if not self.is_recording:
            self.record_mutex.unlock()
            return
            
        self.is_recording = False
        pipe = self.record_pipeline
        self.record_pipeline = None
        self.record_mutex.unlock()

        # Finalize in a background thread because Gst.State.NULL is a blocking call
        def finalize():
            if pipe:
                print("Finalizing MP4 file in background...")
                # Send End-of-Stream (EOS) so the MP4 muxer writes the file header
                pipe.send_event(Gst.Event.new_eos())
                
                # Wait 1s for the file to 'seal' on disk before killing the pipeline
                time.sleep(1.0)
                pipe.set_state(Gst.State.NULL)
                print("Recording saved and finalized successfully.")

        threading.Thread(target=finalize, daemon=True).start()

    def is_person_in_view(self):
    # Returns the AI detection status from the background worker
        return self.ai_worker.get_human_status()

    def start_remote_hardware(self):
        if not self.host_ip: return
        
        # TODO: Fix Robot Microphone Issue

        # --- IMPROVED DYNAMIC CARD DISCOVERY ---
        # We search for 'USB Audio' which appeared in your aplay -l log.
        # This will set CARD_ID to 1, 3, or whatever it currently is.
        find_card = (
            "export PATH=$PATH:/usr/bin:/usr/local/bin:/bin; "
            "CARD_ID=$(aplay -l | grep 'USB Audio' | head -n 1 | cut -d' ' -f2 | tr -d ':'); "
            "if [ -z \"$CARD_ID\" ]; then CARD_ID=1; fi" # Default fallback
        )

        mic_setup = "amixer -c $CARD_ID cset name='Mic Capture Volume' 16; amixer -c $CARD_ID cset name='Mic Capture Switch' on "

        # Cleanup kills previous streams and releases the specific card
        cleanup = (
            f"ssh -o ConnectTimeout=3 quadconn@{self.robot_ip} "
            f"'{find_card}; sudo pkill -9 camera_stream; sudo pkill -9 gst-launch-1.0; "
            f"sudo fuser -k /dev/snd/pcmC${{CARD_ID}}D0p || true'" 
        )
        subprocess.run(cleanup, shell=True)
        
        # Robot -> Host (Microphone)
        launch_audio_send = (
            f"gst-launch-1.0 alsasrc device=plughw:$CARD_ID ! queue ! audioconvert ! audioresample ! "
            f"audio/x-raw,rate=48000,channels=1 ! opusenc bitrate=64000 ! rtpopuspay ! "
            f"udpsink host={self.host_ip} port={MIC_TO_SPEAKER_PORT}"
        )
        
        # Host -> Robot (Speaker)
        # --- FIXED: Changed 'device=plughw:1' to 'device=plughw:$CARD_ID' ---
        launch_audio_recv = (
            f"gst-launch-1.0 udpsrc port={SPEAKER_TO_MIC_PORT} ! "
            "application/x-rtp,media=audio,clock-rate=48000,encoding-name=OPUS,payload=96 ! "
            "rtpjitterbuffer latency=200 ! rtpopusdepay ! opusdec ! "
            "audioconvert ! audioresample ! queue ! alsasink device=plughw:$CARD_ID"
        )

        full_launch = (
            f"ssh -o ConnectTimeout=3 quadconn@{self.robot_ip} "
            f"'({find_card}; {self.robot_exec} {self.host_ip} & {launch_audio_send} & {launch_audio_recv})'"
        )
        subprocess.Popen(full_launch, shell=True)

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
        self.stop_recording()
        subprocess.run(f"ssh -o ConnectTimeout=2 quadconn@{self.robot_ip} \"sudo pkill -9 camera_stream; sudo pkill -9 gst-launch-1.0\"", shell=True)
        self.quit()

from gamepad_data import GamepadData

class ControllerThread(QThread):
    # Signals to update the GUI status bar and the visual controller dashboard
    status_signal = pyqtSignal(str)
    controller_state = pyqtSignal(dict)

    def __init__(self, robot_ip):
        super().__init__()
        self.robot_ip = robot_ip
        self.running = True
        self.daemon = True # Thread terminates automatically if the main application closes
        self.port = 3007   # UDP port defined in the robot's control firmware
        self.joystick = None

        # Physical limit constants synchronized with quad_config.hpp
        # These values define the maximum meters/second and radians/second allowed
        self.MAX_VX = 0.25
        self.MAX_VY = 0.25
        self.MAX_YAW = 0.45

    def _deadzone(self, val):
        # Filters out tiny movements near the center to prevent stick drift
        return 0.0 if abs(val) < 0.05 else val

    def run(self):
        # SDL dummy driver allows Pygame to run without a window inside a background thread
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
        
        try:
            # Full reset of the SDL subsystem to resolve conflicts between threads
            pygame.quit() 
            pygame.init()
            pygame.joystick.init()
        except Exception as e:
            print(f"SDL Subsystem Error: {e}")
            self.status_signal.emit("ERROR")
            return

        # Abort if no physical controller is plugged in
        if pygame.joystick.get_count() == 0:
            self.status_signal.emit("DISCONNECTED")
            return

        try:
            # Bind to the first detected joystick
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            
            # Print hardware stats to verify the controller mapping is correct
            num_axes = self.joystick.get_numaxes()
            num_btns = self.joystick.get_numbuttons()
            print(f"DEBUG: Controller Hardware Detected -> Axes: {num_axes}, Buttons: {num_btns}")
            
            self.status_signal.emit(f"CONNECTED: {self.joystick.get_name()}")
        except Exception as e:
            self.status_signal.emit("ERROR")
            print(f"Joystick Bind Error: {e}")
            return

        # Prepare the UDP socket for low-latency transmission to the robot
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        loop_count = 0

        while self.running:
            # Pump events to refresh the joystick's internal register state
            pygame.event.pump()
            for event in pygame.event.get():
                pass # Clearing the event queue to prevent lag

            # Helper to safely check button states without crashing on missing indices
            btn = lambda idx: self.joystick.get_button(idx) if idx < self.joystick.get_numbuttons() else 0

            # Left Stick: Axis 0 (X) and Axis 1 (Y)
            # We invert the Y-axis (-val) so that pushing 'Up' results in a positive value
            raw_lx = self._deadzone(self.joystick.get_axis(0))
            raw_ly = self._deadzone(-self.joystick.get_axis(1)) 
            
            # Right Stick: Axis 2 (X) and Axis 3 (Y)
            raw_rx = self._deadzone(self.joystick.get_axis(2))
            raw_ry = self._deadzone(-self.joystick.get_axis(3))
            
            # Triggers: Rescale from the default [-1 to 1] range to a clean [0 to 1] range
            raw_lt = (self.joystick.get_axis(4) + 1.0) / 2.0 if self.joystick.get_numaxes() > 4 else 0.0
            raw_rt = (self.joystick.get_axis(5) + 1.0) / 2.0 if self.joystick.get_numaxes() > 5 else 0.0

            # Compute velocities by multiplying raw stick percentage by the MAX limit constants
            # vx = forward/back, vy = strafe left/right, yaw = rotation
            vx = raw_ly * self.MAX_VX
            vy = raw_lx * self.MAX_VY
            yaw_rate = raw_rx * self.MAX_YAW

            # Construct the state dictionary used for both UI visualization and robot command
            state = {
                "lx": raw_lx, "ly": raw_ly, 
                "rx": raw_rx, "ry": raw_ry,
                "LT": raw_lt, "RT": raw_rt,
                "vx": vx, "vy": vy, "yaw_rate": yaw_rate,
                "A": btn(0), "B": btn(1), "X": btn(2), "Y": btn(3),
                "LB": btn(9), "RB": btn(10), 
                "Select": btn(4), "Start": btn(6),
                "L3": btn(7), "R3": btn(8), "Home": btn(5),
                "UP": btn(11), "DOWN": btn(12), "LEFT": btn(13), "RIGHT": btn(14)
            }

            # Print a status heartbeat to the terminal every 1 second (100 loops * 10ms)
            if loop_count % 100 == 0:
                active_inputs = [k for k, v in state.items() if isinstance(v, float) and abs(v) > 0.1]
                active_btns = [k for k, v in state.items() if isinstance(v, int) and v == 1]
                if active_inputs or active_btns:
                    print(f"SUCCESS: Active Inputs -> {active_inputs} {active_btns}")

            # Send the updated state to the Main GUI to move the virtual sliders/icons
            self.controller_state.emit(state)
            
            try:
                # Serialize the state dictionary into the custom GamepadData struct format
                data = GamepadData(**state)
                # Ship the data packet to the robot via UDP
                sock.sendto(bytes(data), (self.robot_ip, self.port))
            except:
                # Silently catch network drops to keep the control thread alive
                pass

            # Maintain a 100Hz update rate (standard for robotics control)
            self.msleep(10)
            loop_count += 1

        # Clean shutdown and release of SDL hardware resources
        sock.close()
        pygame.joystick.quit()
        pygame.display.quit() 

    def stop(self):
        # Stop flag triggered by the GUI's CloseEvent
        self.running = False
        self.wait()