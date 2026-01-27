#!/bin/bash
# Build script for edge_tts_rbnx

set -e

echo "Building edge_tts_rbnx package..."
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select edge_tts_rbnx

echo "edge_tts_rbnx build completed successfully!"
