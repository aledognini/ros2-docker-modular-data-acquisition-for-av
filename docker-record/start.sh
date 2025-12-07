#!/bin/bash

IMAGE_NAME="recording-image"
CONTAINER_NAME="recording_container"

echo "Pulling the latest image: $IMAGE_NAME..."
# docker pull $IMAGE_NAME

# Allow GUI access
xhost +local:root

# Check if the container exists
if docker ps -a | grep -q $CONTAINER_NAME; then
    echo "Container $CONTAINER_NAME exists."

    # Check if the container is running
    if [ "$(docker inspect -f {{.State.Running}} $CONTAINER_NAME)" == "true" ]; then
        echo "Container $CONTAINER_NAME is running. Stopping it now..."
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    else
        echo "Container $CONTAINER_NAME is not running."
        docker rm $CONTAINER_NAME
    fi
else
    echo "Container $CONTAINER_NAME does not exist."
fi

echo "Starting new container ${CONTAINER_NAME}..."



docker run -it --runtime nvidia \
    --privileged \
    --name ${CONTAINER_NAME} \
    --network=host \
    --ipc=host \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v /media/ssd/ros_bags:/root/bags_data \
    -v /tmp/argus_socket:/tmp/argus_socket \
    -v /var/nvidia/nvcam/settings/:/var/nvidia/nvcam/settings/ \
    -v /var/nvidia/nvcam/camera-override.isp:/var/nvidia/nvcam/camera-override.isp \
    ${IMAGE_NAME}
