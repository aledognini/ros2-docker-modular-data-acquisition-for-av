FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Rome
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

#to pass from FastDDS to CycloneDDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml


# Configure ROS 2 repositories
RUN apt-get update && apt-get install -y curl gnupg lsb-release && rm -rf /var/lib/apt/lists/*
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install system dependencies
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    tmux \
    nano \
    tree \
    git \
    cmake \
    python3-pip \
    ros-dev-tools \
    # Dependencies for microstrain_inertial packages found by rosdep
    ros-humble-rclcpp \
    ros-humble-rclcpp-lifecycle \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-nmea-msgs \
    ros-humble-rtcm-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    libgeographic-dev \
    ros-humble-diagnostic-aggregator \
    ros-humble-xacro \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rviz2 \
    ros-humble-rviz-imu-plugin \
    ros-humble-rqt-gui \
    ros-humble-rqt-gui-py \
    # install CycloneDDS
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
    

# Crea la directory per la config DDS
RUN mkdir -p /etc/ros
COPY ./docker/cyclonedds.xml /etc/ros/cyclonedds.xml

# Create the ROS workspace
WORKDIR /root/ROS2_ws


COPY ./ROS2_ws/src ./src
COPY ./data /root/data

# Install dependencies for all packages in the workspace
RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y


# Build the entire workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Source the workspace in.bashrc for interactive sessions
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ROS2_ws/install/local_setup.bash" >> /root/.bashrc

CMD ["bash"]
