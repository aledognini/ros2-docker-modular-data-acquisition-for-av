# Immagive di partenza di ZED SDK
FROM stereolabs/zed:4.2-tools-devel-jetson-jp6.0.0

# Diciamo a Docker di usare /bin/bash per tutti i comandi
SHELL ["/bin/bash", "-c"]

# Imposto l'ambiente
ENV DEBIAN_FRONTEND=noninteractive

#to pass from FastDDS to CycloneDDS
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/ros/cyclonedds.xml

# Installazione Dipendenze (AGGIUNGI QUA ALTRE DIPENDENZE)
RUN apt-get update && apt-get install -y curl gnupg lsb-release git tmux nano tree

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update
RUN apt-get install -y ros-humble-desktop ros-dev-tools ros-humble-rmw-cyclonedds-cpp

RUN rosdep init && rosdep update

# Crea la directory per la config DDS
RUN mkdir -p /etc/ros
COPY ./docker/cyclonedds.xml /etc/ros/cyclonedds.xml

# Setup del Workspace e Compilazione
WORKDIR /root/ROS2_ws

COPY ./ROS2_ws/src ./src
COPY ./data /root/data

# Attiviamo l'ambiente ROS e compiliamo
RUN source /opt/ros/humble/setup.bash && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "libopencv-dev libzstd-dev" && \
    colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release


RUN echo "source /root/ROS2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
