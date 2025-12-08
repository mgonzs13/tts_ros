ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-core AS deps

# Install system dependencies early for better caching
RUN apt update && apt install -y --no-install-recommends \
    git \
    build-essential \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Create ros2_ws and copy files
WORKDIR /root/ros2_ws
COPY . /root/ros2_ws/src

# Clone additional repository
RUN git clone https://github.com/mgonzs13/audio_common.git /root/ros2_ws/src/audio_common

# Install ROS dependencies
RUN rosdep init && rosdep update --include-eol-distros
RUN rosdep install --from-paths src --ignore-src -r -y

# Install Python packages
RUN if [ "$ROS_DISTRO" = "jazzy" ] || [ "$ROS_DISTRO" = "rolling" ]; then \
    pip3 install -r src/requirements.txt --break-system-packages --ignore-installed; \
    else \
    pip3 install -r src/requirements.txt; \
    fi

FROM deps AS builder

# Build the workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# Source the ROS 2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["bash"]
