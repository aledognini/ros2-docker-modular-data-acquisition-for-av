#!/bin/bash

IMAGE_NAME="recording-image"

docker build --network=host -t ${IMAGE_NAME} -f Dockerfile ../

