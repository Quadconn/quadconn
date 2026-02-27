#!/bin/bash

# 1. Export GStreamer Paths (Crucial for the embedded video feed)
export GST_PLUGIN_PATH="/opt/homebrew/lib/gstreamer-1.0"
export GST_PLUGIN_SCANNER="/opt/homebrew/opt/gstreamer/libexec/gstreamer-1.0/gst-plugin-scanner"

# 2. Add Homebrew to the system path
export PATH="/opt/homebrew/bin:$PATH"

# 3. Launch using the Virtual Environment's Python
# This ensures ultralytics and other libraries are found
./venv/bin/python gui.py