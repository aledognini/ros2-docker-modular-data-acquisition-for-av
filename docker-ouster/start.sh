#!/bin/bash

IMAGE_NAME="ouster-ros2-image"
CONTAINER_NAME="ouster_ros2_container"

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



docker run -it \
    --name ${CONTAINER_NAME} \
    -v /media/ssd/ros_bags:/root/bags_data \
    -v /media/ssd/scripts:/root/scripts \
    --network=host \
    --ipc=host \
    --rm \
    --privileged \
    -v /dev:/dev \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    ${IMAGE_NAME}
