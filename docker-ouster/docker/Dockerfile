FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Rome
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

ENV FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
# libpcap-dev, libcurl4-openssl-dev: dipendenze del driver Ouster per la rete e HTTP [1, 2]
# iproute2, net-tools, httpie: utili strumenti di rete per il debug
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    git \
    tmux \
    nano \
    tree \
    python3-pip \
    # Ouster SDK C++ Dependencies
    libeigen3-dev \
    libjsoncpp-dev \
    libtins-dev \
    libpcap-dev \
    libcurl4-openssl-dev \
    libglfw3-dev \
    libglew-dev \
    libpng-dev \
    libflatbuffers-dev \
    libssl-dev \
    # ROS 2 Development Tools
    ros-dev-tools \
    # ROS 2 Perception & Visualization Stack
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    # Clean up apt cache to reduce image size
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Create the ROS workspace
WORKDIR /root/ROS2_ws

COPY ./ROS2_ws/src ./src

# Install dependencies for all packages in the workspace
RUN apt-get update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the entire workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Source the workspace in.bashrc for interactive sessions
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ROS2_ws/install/local_setup.bash" >> /root/.bashrc

CMD ["bash"]
