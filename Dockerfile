# ROS2 Jazzy + Gazebo Harmonic Docker Image for Navigation Testing
FROM osrf/ros:jazzy-desktop-full

# Avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install Gazebo Harmonic and Nav2
RUN apt-get update && apt-get install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-simple-commander \
    ros-jazzy-navigation2 \
    ros-jazzy-ros-gz \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-xacro \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-rviz2 \
    ros-jazzy-rmw-cyclonedds-cpp \
    python3-pip \
    python3-colcon-common-extensions \
    mesa-utils \
    libgl1 \
    libegl1 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Use FastDDS with UDP transport (avoids shared memory issues)
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/delivery_robot_project/fastdds_profile.xml

# Set up workspace
WORKDIR /ros2_ws
COPY . /ros2_ws/src/delivery_robot_project

# Build the workspace
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install"

# Source the workspace in bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc && \
    echo "export FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/src/delivery_robot_project/fastdds_profile.xml" >> ~/.bashrc

# Entry point
COPY docker_entrypoint.sh /docker_entrypoint.sh
RUN chmod +x /docker_entrypoint.sh
ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]

