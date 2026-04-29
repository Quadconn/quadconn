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
    # Captures local Mac microphone audio and sends it to the robot's speakers on Port 3005
    def __init__(self, robot_ip):
        super().__init__()
        self.robot_ip = robot_ip
        self.pipeline = None
        self.daemon = True

    def run(self):
        # GStreamer Launch String -> captures local mic using osxaudiosrc and streams to robot's IP
        gst_str = (
            "osxaudiosrc ! audioconvert ! audioresample ! "
            "opusenc bitrate=64000 ! rtpopuspay ! "
            f"udpsink host={self.robot_ip} port={SPEAKER_TO_MIC_PORT}"
        )
        self.pipeline = Gst.parse_launch(gst_str)
        self.pipeline.set_state(Gst.State.PLAYING)
        
        self.loop = GLib.MainLoop()
        self.loop.run()

    def set_mute(self, is_muted):
        # Toggles the pipeline state between NULL and PLAYING to mute the microphone
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL if is_muted else Gst.State.PLAYING)
            # "ssh sudo systemctl enable [SERVICE]"

    def stop(self):
        # Safely stops the GLib loop and releases the GStreamer hardware
        if self.loop: self.loop.quit()
        if self.pipeline: self.pipeline.set_state(Gst.State.NULL)

# Import your partner's engine files
from FastSlam import ParticleFilter
from OccupancyGrid import OccupancyGrid

class LidarSLAMWorker(QThread):
    # Updated Signal: Expects the Map, and the robot's X, Y, and Theta
    map_updated = pyqtSignal(np.ndarray, float, float, float)

    def __init__(self):
        super().__init__()
        self.running = True
        self.port = 6000
        # Current motion state from GUI (optional fallback)
        self.current_speed = 0.0
        self.current_orientation = 0.0
        # Initialize SLAM parameters from your partner's new plotter
        self.init_params()

    def init_params(self):
    # --- UPDATED FOR HANDHELD MAPPING ---
    # 1. unitGridSize: Try 0.2 (approx 2.4 inches) for higher resolution
        self.og_params = [150, 150, None, 0.2, math.radians(120), 164.042, 300, 1.0]
    
    # 2. scanMatchSearchRadius: Increase to 3.0 or 4.0 feet
    # This allows the SLAM to "catch" you if you walk at a normal human pace.
    # 3. scanSigmaInNumGrid: Increase to 3 or 4 to make the matching "sticker."
        self.sm_params = [3.0, 0.1, 4, 0.3, 0.5, 0.2, 0.15, 5]
    
    # 4. Particles: Bump this to 15 or 20. 
    # Your MacBook Pro can handle it, and more particles = better "guesses" of your position.
        self.pf = None

    def run(self):
        # UDP Setup
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', self.port))
        sock.settimeout(1.0)
        
        count = 0
        last_scan_time = None
        
        print(f"SLAM Engine: Listening on Port {self.port}...")

        while self.running:
            try:
                data, _ = sock.recvfrom(65507)
                scan_entry = json.loads(data.decode('utf-8'))
                
                # Initialization on first packet
                if self.pf is None:
                    self.og_params[2] = scan_entry # Set the initXY from first scan
                    self.pf = ParticleFilter(8, self.og_params, self.sm_params)
                    print("SLAM: Particle Filter Initialized.")

                count += 1
                now = time.monotonic()
                dt = (now - last_scan_time) if last_scan_time is not None else 0.0
                last_scan_time = now

                # Extract motion from scan_entry (provided by sf45_collector.py)
                speed = scan_entry.get('speed', 0.0) 
                orientation = scan_entry.get('theta', 0.0)
                
                # Create motion dict if robot is actually moving
                motion = {'speed': speed, 'orientation': orientation, 'dt': dt} if abs(speed) > 1e-3 else None

                # Step 1: Update Particle Filter
                self.pf.updateParticles(scan_entry, count, motion)
                
                # Step 2: Check for Resampling
                if self.pf.weightUnbalanced():
                    self.pf.resample()

                # --- Step 3: Every 3 frames, send the SLICED map to GUI ---
                if count % 3 == 0:
                    best = max(self.pf.particles, key=lambda p: p.weight)
                    curr_x = best.xTrajectory[-1]
                    curr_y = best.yTrajectory[-1]
                    curr_theta = best.prevMatchedReading['theta']

                    og = best.og
                    
                    # --- THE FIX: SLICE THE MAP AROUND THE ROBOT ---
                    # We define a "window" of 50ft around the robot
                    view_half = 50.0 
                    x_range = [curr_x - view_half, curr_x + view_half]
                    y_range = [curr_y - view_half, curr_y + view_half]

                    # Clip the window so we don't go outside the grid bounds
                    x_range = [max(x_range[0], og.mapXLim[0]), min(x_range[1], og.mapXLim[1])]
                    y_range = [max(y_range[0], og.mapYLim[0]), min(y_range[1], og.mapYLim[1])]

                    # Convert those real-world feet to grid indices
                    x_idx, y_idx = og.convertRealXYToMapIdx(x_range, y_range)

                    # Extract just that piece of the map
                    visited = og.occupancyGridVisited[y_idx[0]:y_idx[1], x_idx[0]:x_idx[1]]
                    total   = og.occupancyGridTotal  [y_idx[0]:y_idx[1], x_idx[0]:x_idx[1]]

                    with np.errstate(divide='ignore', invalid='ignore'):
                        ratio = np.where(total > 0, visited / total, 0.5)
                    
                    # This ogMap is now "Zoomed In" on the robot!
                    ogMap = np.flipud(1.0 - ratio)

                    # Emit the zoomed map and current position
                    self.map_updated.emit(ogMap, curr_x, curr_y, curr_theta)

            except socket.timeout:
                continue
            except Exception as e:
                print(f"SLAM Error: {e}")

    def update_motion(self, speed, orientation):
        """Update the motion model based on stick inputs from the GUI."""
        self.current_speed = speed
        self.current_orientation = orientation

    def stop(self):
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
        # Triggers the robot's camera binary and two-way audio streams via SSH
        if not self.host_ip: return
        print(f"Connecting to robot at {self.robot_ip}...")
        
        # Dynamically locate the USB microphone hardware card ID
        find_card = (
            "export PATH=$PATH:/usr/bin:/usr/local/bin:/bin; "
            "CARD_ID=$(arecord -l | grep \"USB PnP Sound Device\" | cut -d\" \" -f2 | tr -d \":\"); "
            "if [ -z \"$CARD_ID\" ]; then CARD_ID=2; fi"
        )

       # Apply maximum hardware gain and unmute the audio input
        mic_setup = "amixer -c $CARD_ID cset name=\"Mic Capture Volume\" 16; amixer -c $CARD_ID cset name=\"Mic Capture Switch\" on "

        # Force kill video and old GStreamer audio pipes
        cleanup = (
            f"ssh -o ConnectTimeout=3 quadconn@{self.robot_ip} "
            f"'{find_card}; {mic_setup}; sudo pkill -9 camera_stream; sudo pkill -9 gst-launch-1.0'"
        )
        subprocess.run(cleanup, shell=True)
        
        # Define binary execution command for camera stream
        launch_video = f"{self.robot_exec} {self.host_ip}"
        
        # Robot Mic -> Host (Port 3004)
        audio_caps = "audio/x-raw,rate=48000,channels=1"
        launch_audio_send = (
            f"gst-launch-1.0 alsasrc device=plughw:$CARD_ID ! audioconvert ! audioresample ! "
            f"{audio_caps} ! opusenc bitrate=64000 ! rtpopuspay ! "
            f"udpsink host={self.host_ip} port={MIC_TO_SPEAKER_PORT}"
        )
        
        # Host -> Robot Speakers (Port 3005)
        launch_audio_recv = (
            f"gst-launch-1.0 udpsrc port={SPEAKER_TO_MIC_PORT} ! "
            "application/x-rtp,media=audio,clock-rate=48000,encoding-name=OPUS,payload=96 ! "
            "rtpopusdepay ! opusdec ! audioconvert ! audioresample ! alsasink device=plughw:1"
        )

        # Trigger the integrated video and audio processes in a parallel background session
        full_launch = (
            f"ssh -o ConnectTimeout=3 quadconn@{self.robot_ip} "
            f"'({find_card}; {launch_video} & {launch_audio_send} & {launch_audio_recv})'"
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
    status_signal = pyqtSignal(str)
    controller_state = pyqtSignal(dict)

    def __init__(self, robot_ip):
        super().__init__()
        self.robot_ip = robot_ip
        self.running = True
        self.daemon = True
        self.port = 3007 
        self.joystick = None

    def _deadzone(self, joystick_input):
        # Applies the requested 0.05 threshold to prevent "stick drift"
        return 0.0 if abs(joystick_input) < 0.05 else joystick_input

    def run(self):
        # 1. Environment setup MUST happen before any pygame.init calls
        os.environ["SDL_VIDEODRIVER"] = "dummy"
        os.environ["PYGAME_HIDE_SUPPORT_PROMPT"] = "hide"
        
        try:
            # 2. Initialize the display FIRST (using the dummy driver)
            # This fixes the "video system not initialized" error
            pygame.display.init() 
            pygame.joystick.init()
        except Exception as e:
            print(f"SDL Subsystem Error: {e}")
            self.status_signal.emit("ERROR")
            return

        if pygame.joystick.get_count() == 0:
            self.status_signal.emit("DISCONNECTED")
            # Cleanup display before exiting
            pygame.display.quit()
            return

        try:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.status_signal.emit(f"CONNECTED: {self.joystick.get_name()}")
        except Exception as e:
            self.status_signal.emit("ERROR")
            print(f"Joystick Bind Error: {e}")
            return

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        while self.running:
            pygame.event.pump()

            # Helper for buttons based on Flydigi APEX 4 mapping
            btn = lambda idx: self.joystick.get_button(idx) if idx < self.joystick.get_numbuttons() else 0

            # Stick Mapping
            lx = self._deadzone(self.joystick.get_axis(0))
            ly = self._deadzone(self.joystick.get_axis(1))
            rx = self._deadzone(self.joystick.get_axis(2))
            ry = self._deadzone(self.joystick.get_axis(3))
            
            # Trigger Mapping (0.0 to 1.0)
            lt_raw = self.joystick.get_axis(4) if self.joystick.get_numaxes() > 4 else -1.0
            rt_raw = self.joystick.get_axis(5) if self.joystick.get_numaxes() > 5 else -1.0
            l2 = (lt_raw + 1.0) / 2.0
            r2 = (rt_raw + 1.0) / 2.0

            # D-Pad Mapping (Button Indices: 11-14)
            d_x, d_y = 0, 0
            if btn(11): d_y = 1   # Physical Up
            elif btn(12): d_y = -1 # Physical Down
            
            if btn(13): d_x = -1  # Physical Left
            elif btn(14): d_x = 1   # Physical Right

            # Gamepad Construction
            # All indices aligned with your Flydigi APEX 4 feedback
            data = GamepadData(
                dpad_x = d_x,
                dpad_y = d_y,
                A = btn(0), 
                B = btn(1), 
                X = btn(2), 
                Y = btn(3),
                LB = btn(9),      
                RB = btn(10),     
                Select = btn(4),  
                Start = btn(6),   
                L3 = btn(7),      
                R3 = btn(8),      
                Home = btn(5),    
                lx = lx, ly = ly, rx = rx, ry = ry, LT = l2, RT = r2
            )

            # 1. Put EVERYTHING in the dictionary
            state = {
                "A": btn(0), "B": btn(1), "X": btn(2), "Y": btn(3),
                "LB": btn(9), "RB": btn(10), "Select": btn(4), "Start": btn(6),
                "L3": btn(7), "R3": btn(8), "Home": btn(5),
                "UP": btn(11), "DOWN": btn(12), "LEFT": btn(13), "RIGHT": btn(14),
                "lx": lx, "ly": ly, "rx": rx, "ry": ry,
                "LT": l2, "RT": r2,
                "dpad_x": d_x,
                "dpad_y": d_y
            }
            
            # 2. Create a dictionary of the state to send to the GUI
            self.controller_state.emit(state)

            # 3. Create the data packet for the robot (No more manual LT/RT/dpad here!)
            try:
                # By just using **state, we avoid the "multiple values" error
                data = GamepadData(**state)
                sock.sendto(bytes(data), (self.robot_ip, self.port))
            except Exception as e:
                pass

            self.msleep(10) # 100Hz Polling

        sock.close()
        pygame.joystick.quit()
        pygame.display.quit() # Always cleanup the dummy display

    def stop(self):
        self.running = False
        self.wait()