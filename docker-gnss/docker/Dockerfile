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
    nmap \
    cmake \
    python3-pip \
    ros-dev-tools \
    ros-humble-gps-msgs \
    # install CycloneDDS
    ros-humble-rmw-cyclonedds-cpp \
    libserialport-dev && \
    rm -rf /var/lib/apt/lists/*
    
# INSTALL PYTHON LIBRARIES FOR RTK CORRECTIONS
RUN pip3 install paho-mqtt pyserial

# Compile and install the external dependency 'libsbp'
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

# Create the ROS workspace

# Crea la directory per la config DDS
RUN mkdir -p /etc/ros
COPY ./docker/cyclonedds.xml /etc/ros/cyclonedds.xml

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
