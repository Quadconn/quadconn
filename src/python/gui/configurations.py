### CONFIGURATIONS ###

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
    print("Global Torch Engine Initialized Successfully")
except Exception as e:
    # Safe to ignore on macOS or environments without Torch installed
    print(f"Global Setup Note: {e}")

# Configuration Constants
TELEMETRY_PORT = 808
VIDEO_PORT = 5000
MIC_TO_SPEAKER_PORT = 3004
SPEAKER_TO_MIC_PORT = 3005
VIDEO_RECORD_PORT = 5001
AUDIO_RECORD_PORT = 3006
YOLO_WEIGHTS = 'yolov8n.pt'
VOLT_WARNING = 15.2
VOLT_CRITICAL = 14.4
POWER_LIMIT_W = 300.0
LIDAR_UDP_PORT = 6000
LIDAR_MAX_RANGE_FT = 164.0
LIDAR_SAMPLES = 300
SCAN_FOV_DEG = 120.0

# Robot Hardware Info
ROBOT_IP = "100.81.189.79"
ROBOT_EXEC = "/home/quadconn/gui_branch/quadconn/build/camera_stream"