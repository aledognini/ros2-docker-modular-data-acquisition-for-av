FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Rome
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

#to pass from FastDDS to CycloneDDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml

# dipendenze
RUN apt-get update && apt-get install -y \
    python3-pip \
    iproute2 \
    can-utils \
    git \
    tmux \
    tree \
    nano \
    ros-humble-rviz2 \
    # install CycloneDDS
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir \
    cantools \
    python-can
    
# INSTALL PYTHON LIBRARIES FOR RTK CORRECTIONS
RUN pip3 install paho-mqtt pyserial

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
COPY ./data /root/data

# Installo le dipendenze dei pacchetti ROS2
RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y

# Compilo il workspace
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install



RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /root/ROS2_ws/install/local_setup.bash" >> /root/.bashrc

CMD ["bash"]

