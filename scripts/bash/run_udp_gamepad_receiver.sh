#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
source "$SCRIPT_DIR/common.sh"

TARGET_SCRIPT="$PROJECT_ROOT/src/python/udp_gamepad_receiver.py"

# executable
exec "$VENV_PYTHON" "$TARGET_SCRIPT" "$@"