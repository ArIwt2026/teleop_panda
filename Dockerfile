FROM osrf/ros:humble-desktop

# Prevent interactive prompts during apt install
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-websocket \
    libpoco-dev \
    libeigen3-dev \
    libyaml-cpp-dev \
    libopencv-dev \
    libomp-dev \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble packages
RUN apt-get update && apt-get install -y \
    ros-humble-control-msgs \
    ros-humble-xacro \
    ros-humble-angles \
    ros-humble-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \
    ros-humble-controller-manager \
    ros-humble-hardware-interface \
    ros-humble-generate-parameter-library \
    ros-humble-controller-interface \
    ros-humble-ros2-control-test-assets \
    ros-humble-moveit \
    ros-humble-cv-bridge \
    ros-humble-rviz2 \
    ros-humble-rosbridge-server \
    ros-humble-librealsense2 \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    ros-humble-v4l2-camera \
    ros-humble-sick-safetyscanners-base \
    ros-humble-sick-safetyscanners2 \
    ros-humble-sick-safetyscanners2-interfaces \
    ros-humble-backward-ros \
    ros-humble-joint-state-publisher \
    ros-humble-ament-cmake-clang-format \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-trajectory-controller \
    ros-humble-pilz-industrial-motion-planner \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace

# Clone missing olv_module_descriptions dependency
RUN git clone https://github.com/olive-robotics/olvx_descriptions_module.git /workspace/src/olv_module_descriptions

# Clone and build libfranka dependency
RUN git clone --branch 0.9.2 --depth 1 --recurse-submodules https://github.com/frankaemika/libfranka.git /workspace/libfranka && \
    mkdir -p /workspace/libfranka/build && \
    cd /workspace/libfranka/build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=ON -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    touch /workspace/libfranka/COLCON_IGNORE

# Copy workspace source files (respecting .dockerignore)
COPY . /workspace/src/teleop_panda

# Build ROS 2 workspace — scan all package trees under src/
RUN . /opt/ros/humble/setup.sh && \
    colcon build \
        --base-paths src \
        --allow-overriding franka_msgs franka_teach_msgs \
        --ignore-src \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# The direct Cartesian mode is a standalone libfranka ROS node. Build it
# explicitly so a fresh image contains its executable and launch file.
RUN . /opt/ros/humble/setup.sh && \
    . /workspace/install/setup.sh && \
    colcon build \
        --base-paths src/teleop_panda \
        --packages-select franka_debug_tools \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# Default command to run
CMD ["bash"]
