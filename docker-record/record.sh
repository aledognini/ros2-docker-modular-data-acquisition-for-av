#!/bin/bash

# Script to easily record specific topics into a ROS2 bag file

# The name of the container where our ROS nodes are running
CONTAINER_NAME="recording_container"

# Check if the container is actually running before trying to record
if ! docker ps -q -f name=${CONTAINER_NAME} | grep -q .; then
    echo "Error: The container '${CONTAINER_NAME}' is not running."
    echo "Please start the system first with ./start.sh and run the nodes."
    exit 1
fi

# Define a name for the bag file, including the current date and time
# This ensures every recording has a unique name.
BAG_NAME="bag_$(date +%Y-%m-%d_%H-%M-%S)"

# List all the topics you want to record.
# Using a single variable makes the final command cleaner.
TOPICS_TO_RECORD="/can_bus/scu__steering_wheel_angle \
/imu/data_can \
/can_bus/ccu__horn \
/can_bus/ref__horn \
/can_bus/ccu__turning_light_l \
/can_bus/ccu__turning_light_r \
/can_bus/ref__turning_light_l \
/can_bus/ref__turning_light_r \
/vehicle/velocity \
/cmd/ref_steering_wheel_angle \
/cmd/ref_throttle\
/cmd/ref_brake_pressure \
/cmd/ref_car_speed \
/cmd/ref_ax \
/cmd/ref_steering_wheel_enable \
/cmd/ref_throttle_enable \
/cmd/ref_brakes_enable \
/cmd/ref_speed_control_enable \
/cmd/ref_turning_light_l \
/cmd/ref_turning_light_r \
/cmd/ref_drive_neutral_reverse \
/cmd/ref_lights \
/cmd/ref_horn \
/cmd/ref_key \
/cmd/ref_doors \
/cmd/ref_party_mode \
/pedestrian_warning \
/tf \
/tf_static \
/zed/robot_description \
/zed/zed_node/left/image_rect_color/compressed \
/zed/zed_node/left/camera_info \
/zed/zed_node/right/image_rect_color/compressed \
/zed/zed_node/right/camera_info \
/zed/zed_node/depth/depth_registered/compressedDepth \
/zed/zed_node/depth/camera_info \
/ouster/metadata \
/ouster/points \
/ouster/imu \
/imu/data \
/antenna_front/timereference \
/antenna_front/imu \
/antenna_front/twistwithcovariancestamped \
/antenna_front/navsatfix \
/antenna_rear/timereference \
/antenna_rear/imu \
/antenna_rear/twistwithcovariancestamped \
/antenna_rear/navsatfix \
/rosout"

echo "--- Starting ROS2 Bag Recording ---"
echo "Recording the following topics:"
echo "${TOPICS_TO_RECORD}"
echo ""
echo "Bag file will be saved inside the container at: /root/bags_data/${BAG_NAME}"
echo "Press Ctrl+C in THIS terminal to stop recording."
echo "-------------------------------------"

# Execute the ros2 bag record command inside the running container
docker exec -it ${CONTAINER_NAME} bash -c "source /opt/ros/humble/setup.bash && source /root/ROS2_ws/install/local_setup.bash && ros2 bag record -o /root/bags_data/${BAG_NAME} ${TOPICS_TO_RECORD}"

echo "--- Recording stopped. ---"
