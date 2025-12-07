FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Rome
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

#to pass from FastDDS to CycloneDDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml

# ALL THE DEPENDENCIES
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    cmake \
    git \
    tmux \
    nano \
    tree \
    python3-pip \
    ros-dev-tools \
    # Dipendenze da CAN
    iproute2 \
    can-utils \
    # Dipendenze da GNSS
    nmap \
    libserialport-dev \
    # Dipendenze da Ouster
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
    # Dipendenze da IMU
    libgeographic-dev \
    # Dipendenze ROS da ZED
    ros-humble-desktop \
    # Dipendenze ROS
    ros-humble-rclcpp-lifecycle \
    ros-humble-nav-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-nmea-msgs \
    ros-humble-rtcm-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-diagnostic-aggregator \
    ros-humble-xacro \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-rviz-imu-plugin \
    ros-humble-rqt-gui \
    ros-humble-rqt-gui-py \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-gps-msgs \
    ros-humble-rviz-default-plugins \
    ros-humble-pcl-conversions \
    ros-humble-cv-bridge \
    ros-humble-tf2-tools \
    ros-humble-image-transport-plugins \
    ros-humble-point-cloud-transport-plugins \
    # install CycloneDDS
    ros-humble-rmw-cyclonedds-cpp \
    # tools for advanced testing
    lttng-tools \
    liblttng-ust-dev \
    python3-lttng \
    python3-babeltrace \
    babeltrace \
    && rm -rf /var/lib/apt/lists/*

# Le librerie Python per CAN e GNSS
RUN pip3 install --no-cache-dir \
    cantools \
    python-can \
    paho-mqtt \
    pyserial

# La libreria libsbp per GNSS
RUN git clone https://github.com/swift-nav/libsbp.git /tmp/libsbp && \
    cd /tmp/libsbp && \
    git checkout v4.11.0 && \
    cd c && \
    git submodule update --init --recursive && \
    mkdir build && cd build && \
    cmake -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_STANDARD_REQUIRED=ON -DCMAKE_CXX_EXTENSIONS=OFF ../ && \
    make && \
    make install && \
    rm -rf /tmp/libsbp
    
# INSTALL ros2-performance
WORKDIR /root/performance_ws/src
RUN git clone https://github.com/irobot-ros/ros2-performance.git -b humble && \
    cd ros2-performance && \
    git submodule update --init --recursive
WORKDIR /root/performance_ws
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# SETUP for ros2_tracing
RUN groupadd -f tracing && usermod -a -G tracing root



# creazione del workspace per ROS2

# Crea la directory per la config DDS
RUN mkdir -p /etc/ros
COPY ./docker/cyclonedds.xml /etc/ros/cyclonedds.xml

# Inizializzo rosdep
RUN rosdep init || true && rosdep update

# Imposto la directory di lavoro principale
WORKDIR /root/ROS2_ws

# Copio il codice sorgente e i dati
COPY ./ROS2_ws/src ./src
COPY ./config /root/config

# Installo le dipendenze dei pacchetti ROS2
RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

# Compilo il workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install


RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ROS2_ws/install/local_setup.bash" >> /root/.bashrc

CMD ["bash"]

