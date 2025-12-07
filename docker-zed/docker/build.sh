#!/bin/bash

IMAGE_NAME="zed-ros2-image"

docker build --network=host -t ${IMAGE_NAME} -f Dockerfile ../

