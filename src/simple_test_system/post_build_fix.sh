#!/bin/bash
# Post-build script to ensure executables are in the correct location for launch files

INSTALL_DIR="/home/paramesh502/ros2_ws/install/simple_test_system"
LIB_DIR="$INSTALL_DIR/lib/simple_test_system"
BIN_DIR="$INSTALL_DIR/bin"

# Create lib directory if it doesn't exist
mkdir -p "$LIB_DIR"

# Copy executables from bin to lib directory
if [ -d "$BIN_DIR" ]; then
    cp "$BIN_DIR"/* "$LIB_DIR"/ 2>/dev/null || true
    echo "Executables copied to lib directory"
fi
