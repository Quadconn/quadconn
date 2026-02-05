# Video Streaming Guide

## Building the Executable
build the project using CMake and navigate to the directory 
```bash
cmake -B build
cmake --build build
cd build
```

## Running the File
run the camera_stream executable with the user IP (your Tailscale IP) as an argument
```bash
./camera_stream 100.127.67.67
```

## Configuring the User PC
These commands will be run on **your** device, not the edge computer.
First, ensure that GStreamer is installed on your specific device. 
Instructions are using [this link][https://gstreamer.freedesktop.org/download/?__goaway_challenge=meta-refresh&__goaway_id=90883cf2a62bb803eb9c3473fd14ffb8&__goaway_referer=https%3A%2F%2Fgstreamer.freedesktop.org%2F#windows]

Check if GStreamer is configured using this command:
```bash
gst-launch-1.0 --version
```

Once verified, run the following command:
```bash
gst-launch-1.0 udpsrc port=5000 ! "application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264,payload=(int)96" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

if you do not see a video stream I have failed.

