#!/bin/bash

IMAGE_NAME="ouster-ros2-image"

docker build --network=host -t ${IMAGE_NAME} -f Dockerfile ../

