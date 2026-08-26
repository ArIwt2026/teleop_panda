#!/bin/bash
set -e

# Automatically fallback to software rendering if no GPU device is detected
if [ ! -c "/dev/dri/card0" ] && [ ! -c "/dev/dri/renderD128" ] && [ -z "$NVIDIA_VISIBLE_DEVICES" ]; then
    echo "[Entrypoint] No GPU hardware acceleration device found. Enabling software OpenGL rendering (LIBGL_ALWAYS_SOFTWARE=1)."
    export LIBGL_ALWAYS_SOFTWARE=1
fi

# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Source the workspace setup
if [ -f "/workspace/install/setup.bash" ]; then
    source /workspace/install/setup.bash
fi

exec "$@"
