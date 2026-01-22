#!/bin/bash

# Script to run sMotor_Interface executable and ik.py Python script

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Paths
EXECUTABLE="$PROJECT_ROOT/build/sMotor_Interface"
PYTHON_SCRIPTS_DIR="$PROJECT_ROOT/src/python_scripts"
VENV_DIR="$PYTHON_SCRIPTS_DIR/.venv"
SCRIPT="$PYTHON_SCRIPTS_DIR/ik.py"

# Cleanup function to kill background processes
cleanup() {
    echo ""
    echo "Cleaning up..."
    kill $EXECUTABLE_PID $PID 2>/dev/null || true
    exit 0
}

# Set trap to call cleanup on SIGINT (Ctrl+C)
trap cleanup SIGINT

# Check if executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo "Error: Executable not found at $EXECUTABLE"
    exit 1
fi

# Check if virtual environment exists
if [ ! -d "$VENV_DIR" ]; then
    echo "Error: Virtual environment not found at $VENV_DIR"
    echo "Please create it first with: python3 -m venv $VENV_DIR"
    exit 1
fi

# Check if ik.py exists
if [ ! -f "$SCRIPT" ]; then
    echo "Error: python not found not found at $SCRIPT"
    exit 1
fi

# Start sMotor_Interface in background
echo "Starting sMotor_Interface..."
"$EXECUTABLE" &
EXECUTABLE_PID=$!

# Give executable time to initialize
sleep 1

# Run ik.py in background with virtual environment
echo "Running ik.py..."
source "$VENV_DIR/bin/activate"
python "$SCRIPT" &
PID=$!
deactivate

# Wait for both processes to complete
wait $EXECUTABLE_PID $PID

echo "Done."

