import sys
import cv2
import numpy as np
import os
import paramiko
import time
from PyQt6.QtCore import QThread, pyqtSignal, Qt
from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt6.QtGui import QImage, QPixmap

# Configuration
ROBOT_IP = "100.81.189.79"
ROBOT_USER = "quadconn"
ROBOT_SCRIPT = "/home/quadconn/start_vision.sh"
PRIVATE_KEY_PATH = os.path.expanduser("~/.ssh/quadconn")

# Mute OpenCV/FFmpeg noise and force TCP transport for stability
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
os.environ["OPENCV_LOG_LEVEL"] = "FATAL" 

class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(np.ndarray)

    def run(self):
        rtsp_url = f"rtsp://{ROBOT_IP}:8554/mystream"
        
        while True:
            cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
            
            if not cap.isOpened():
                self.msleep(1000) 
                continue

            print("Stream connected! Feeding video...")
            while True:
                ret, frame = cap.read()
                if ret:
                    self.change_pixmap_signal.emit(frame)
                else:
                    print("Stream lost. Reconnecting...")
                    cap.release()
                    break 

class QuadconnGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quadconn Camera Feed")
        self.setFixedSize(800, 650)
        self.setStyleSheet("background-color: #121212;")

        self.video_label = QLabel(self)
        self.video_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.video_label.setStyleSheet("border: 2px solid #333; background-color: black;")
        
        layout = QVBoxLayout()
        layout.addWidget(self.video_label)
        self.setLayout(layout)
        self.start_thread()

    def start_thread(self):
        self.thread = VideoThread()
        self.thread.change_pixmap_signal.connect(self.update_image)
        self.thread.start()

    def update_image(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        qt_format = QImage(rgb_image.data, w, h, ch * w, QImage.Format.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_format).scaled(780, 580, Qt.AspectRatioMode.KeepAspectRatio)
        self.video_label.setPixmap(pixmap)

    def closeEvent(self, event):
        print("\nClosing Dashboard. Shutting down remote vision...")
        try:
            ssh = paramiko.SSHClient()
            ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            my_key = paramiko.Ed25519Key.from_private_key_file(PRIVATE_KEY_PATH)
            ssh.connect(ROBOT_IP, username=ROBOT_USER, pkey=my_key)
            
            ssh.exec_command("pkill -9 mediamtx && pkill -9 ffmpeg")
            ssh.close()
            print("Remote processes stopped. Hardware standby.")
        except Exception as e:
            print(f"Could not stop remote processes: {e}")
        
        self.thread.terminate()
        event.accept()

def start_remote_services():
    try:
        print(f"Connecting to {ROBOT_IP}...")
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        my_key = paramiko.Ed25519Key.from_private_key_file(PRIVATE_KEY_PATH)
        ssh.connect(ROBOT_IP, username=ROBOT_USER, pkey=my_key)

        print("Booting remote vision systems...")
        ssh.exec_command(f"bash -l -c '{ROBOT_SCRIPT}'")
        
        time.sleep(4) 
        ssh.close()
        print("Robot is ready. Initializing Dashboard...")
        return True
    except Exception as e:
        print(f"Initialization Error: {e}")
        return False

if __name__ == "__main__":
    if start_remote_services():
        app = QApplication(sys.argv)
        gui = QuadconnGUI()
        gui.show()
        sys.exit(app.exec())
    else:
        print("Failed to initialize robot. Check your connection.")