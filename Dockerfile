ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO} AS deps

# Set working directory
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]

# Copy only the src folder
COPY src/ /root/ros2_ws/src/

# Copy local yolo_models folder contents into container
COPY yolo_models/ /root/yolo_models/

# Install dependencies
RUN apt-get update
RUN apt-get -y --quiet --no-install-recommends install python3 python3-pip
RUN rosdep install --from-paths src --ignore-src -r -y
RUN apt-get install ros-${ROS_DISTRO}-ros-gz -y
RUN apt-get install ros-jazzy-gz-ros2-control ros-jazzy-gz-ros2-control-demos ros-jazzy-ros2-controllers -y

RUN if [ "$(lsb_release -rs)" = "24.04" ] || [ "$(lsb_release -rs)" = "24.10" ]; then \
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

# Add the colored prompt for container sessions
RUN echo 'export PS1="\[\e[1;35m\][CONTAINER:ros] \u@\h\[\e[0m\]:\[\e[1;33m\]\w\[\e[0m\]\$ "' >> ~/.bashrc

# Run a default command, e.g., starting a bash shell
CMD ["bash"]
