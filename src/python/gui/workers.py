### WORKERS ###

import socket
import subprocess
import numpy as np
import cv2
import time
import threading
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

    def stop(self):
        # Safely stops the GLib loop and releases the GStreamer hardware
        if self.loop: self.loop.quit()
        if self.pipeline: self.pipeline.set_state(Gst.State.NULL)

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
        
        # Force kill video and old GStreamer audio pipes
        cleanup = (
            f"ssh -o ConnectTimeout=3 quadconn@{self.robot_ip} "
            f"\"sudo pkill -9 camera_stream; sudo pkill -9 gst-launch-1.0; sudo fuser -k /dev/video0\""
        )
        subprocess.run(cleanup, shell=True)
        
        # Launch binary pointing back to this computer
        launch_video = f"{self.robot_exec} {self.host_ip}"
        
        # Robot Mic -> Host (Port 3004)
        launch_audio_send = (
            f"gst-launch-1.0 alsasrc device=hw:WEBCAM ! audioconvert ! audioresample ! "
            f"\\\"audio/x-raw,rate=48000,channels=1\\\" ! opusenc bitrate=64000 ! rtpopuspay ! "
            f"udpsink host={self.host_ip} port={MIC_TO_SPEAKER_PORT}"
        )
        
        # Host -> Robot Speakers (Port 3005)
        launch_audio_recv = (
            f"gst-launch-1.0 udpsrc port={SPEAKER_TO_MIC_PORT} ! "
            f"\\\"application/x-rtp,media=audio,clock-rate=48000,encoding-name=OPUS,payload=96\\\" ! "
            f"rtpopusdepay ! opusdec ! audioconvert ! audioresample ! alsasink device=hw:UACDemoV10"
        )

        # Execute all 3 remote processes in parallel
        full_launch = f"ssh -o ConnectTimeout=3 quadconn@{self.robot_ip} \"({launch_video} & {launch_audio_send} & {launch_audio_recv})\""
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