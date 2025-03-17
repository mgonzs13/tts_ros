ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS deps

# Create ros2_ws and copy files
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
COPY . /root/ros2_ws/src

# Install dependencies
RUN apt-get update
RUN apt-get -y --quiet --no-install-recommends install python3-pip

WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/mgonzs13/audio_common.git

WORKDIR /root/ros2_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash
RUN rosdep update --include-eol-distros && rosdep install --from-paths src --ignore-src -r -y
RUN rosdep install --from-paths src --ignore-src -r -y

RUN if [ "$ROS_DISTRO" = "jazzy" ] || [ "$ROS_DISTRO" = "rolling" ]; then \
    pip3 install -r src/requirements.txt --break-system-packages --ignore-installed; \
    else \
    pip3 install -r src/requirements.txt; \
    fi

# Build the ws with colcon
FROM deps AS builder
ARG CMAKE_BUILD_TYPE=Release
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build

# Source the ROS 2 setup file
RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["bash"]
