#!/usr/bin/env bash

#  Load shared configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
source "$SCRIPT_DIR/common.sh"

TARGET_BIN="$PROJECT_ROOT/build/src/cpp/perception/udp_sender"

#  Safety Check: Find executable
if [ ! -f "$TARGET_BIN" ]; then
    echo "Error: Executable not found at $TARGET_BIN"
    echo "Did you forget to build the project (cmake -B build && cmake --build build)?"
    exit 1
fi

if [ ! -x "$TARGET_BIN" ]; then
    echo "changing permissions to be executable"
    chmod +x "$TARGET_BIN"
fi

#  Execute the binary, replacing the bash process
exec "$TARGET_BIN" "$@"