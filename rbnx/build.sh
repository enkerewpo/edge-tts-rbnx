#!/bin/bash
# Build script for edge_tts_rbnx

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PACKAGE_DIR"

echo "Building edge_tts_rbnx package..."
source /opt/ros/humble/setup.bash

# Clean previous build to avoid "File exists" (Errno 17) with --symlink-install
if [ -d "build" ] || [ -d "install" ]; then
    echo "Cleaning previous build artifacts..."
    rm -rf build install log
fi

colcon build --symlink-install --packages-select edge_tts_rbnx

echo "edge_tts_rbnx build completed successfully!"
