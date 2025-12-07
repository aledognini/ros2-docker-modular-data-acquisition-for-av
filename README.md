# ros2-docker-modular-data-acquisition-for-av
Modular data acquisition architecture for autonomous vehicle (AV) research activities. The system uses Docker containerization for software isolation and ROS2 as the real-time communication middleware. It includes containerized software stacks for LiDAR, Stereo Camera, GNSS, IMU, and CAN-bus. It was validated on a AV of the Politecnico di Milano.

-----

# 📖 HOW TO USE EACH SINGLE CONTAINER

This guide provides the necessary steps to configure and start the individual system containers.

## 0\) Initial Configuration (CycloneDDS)

**⚠️ Before proceeding, you must set the correct configurations for CycloneDDS.**

Execute the following commands in the host terminal:

```bash
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo ip link set lo multicast on
```

-----

## 1\) DUAL-ANTENNA GNSS

To start the GNSS container:

1.  Enter the repository: `/docker-gnss`

2.  Launch the container: `./start.sh`

3.  Inside the `gnss_ros2_container`, launch these commands in separate terminals:

      * **Terminal 1:** `cd /root/scripts && python3 rtk_write.py`
      * **Terminal 2:** `ros2 launch gnss_bringup antenna_front.launch.py`
      * **Terminal 3:** `ros2 launch gnss_bringup antenna_rear.launch.py`

4.  To check the correct functioning, watch the **topic list**.

5.  To publish GPS position on the MQTT broker and get the pedestrian warning signal:

    ```bash
    # Publishes the GPS position on the MQTT broker
    ros2 launch ros2_mqtt_bridge car.launch.py

    # Gets the boolean value for the topic /pedestrian_warning from the MQTT broker
    cd /root/mqtt_script && python3 mqtt_warning_bridge_node.py
    ```

-----

## 2\) CAN-BUS

To configure and start the CAN-BUS container:

1.  **CAN Interface Configuration (Host):**
      * *NB: Use `ip a` to check if `can2` is correct.*
      * Command 1: `sudo ip link set can2 type can bitrate 500000` 
      * [Command 2: `sudo ip link set up can2` 
2.  Enter the repository: `/docker-can`
3.  Launch the container: `./start.sh` 
4.  Inside the `can_ros2_container`, launch these commands in separate terminals:
      * *Functioning check command:* `'candump can2'`
      * **Terminal 1:** `ros2 run can_decoder decoder_node`
      * **Terminal 2:** `ros2 run can_commander commander_node`
5.  To check the correct functioning, watch the **topic list**.
6.  To publish commands to the car, you need to publish on the topics `/cmd/...` (NB: the type of message has to be correct).
7.  **Commander Node Tests:**
      * Test 1: `ros2 run test1_commander test1_command_node` (Controls lights and horn for an initial simple test)
      * Test 2: `ros2 run test2_commander test2_command_node` (Controls the motor and brakes for a more complex test on the can)
8.  **Pedestrian Warning Visualization:**
      * Launch this command to visualize the warning signal for pedestrian detection:
        `ros2 launch pedestrian_visualizer visualizer.launch.py`
      * *To check how the marker changes color:*
        ```bash
        ros2 topic pub /pedestrian_warning std_msgs/msg/Bool "data: true"
        ros2 topic pub /pedestrian_warning std_msgs/msg/Bool "data: false"
        ```

-----

## 3\) ZED CAMERA

To start the ZED Camera container:

1.  Enter the repository: `/docker-zed`

2.  Launch the container: `./start.sh`

3.  Check the connection with the cameras by launching `'ZED_Explorer'` (NB: a reboot is necessary every time the connection with the zed is changed).

4.  Inside the `zed_ros2_container`, launch the appropriate command:

      * **Case 1: Only one ZED camera:**
        ```bash
        ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
        ```
      * **Case 2: Two ZED cameras:** (Launch commands in separate terminals)
        ```bash
        # Terminal 1 (Front)
        ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx camera_name:=zed_front

        # Terminal 2 (Rear)
        ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx camera_name:=zed_rear
        ```

5.  To check the correct functioning, watch the **topic list** and use **rviz2**.

-----

## 4\) IMU

To start the IMU container:

1.  Enter the repository: `/docker-imu`

2.  Launch the container: `./start.sh` 

3.  Inside the `imu_ros2_container`, launch the following command:

    ```bash
    ros2 launch microstrain_inertial_examples cv7_launch.py
    ```

-----

## 5\) LiDAR (Ouster)

To start the LiDAR container:

1.  Enter the repository: `/docker-ouster`
2.  Launch the container: `./start.sh`
3.  Inside the `ouster_ros_container`, launch this command:
    ```bash
    ros2 launch ouster_ros sensor.independent.launch.xml \
        sensor_hostname:=192.168.1.50 \
        azimuth_window_start:=90000 \
        azimuth_window_end:=270000 
    ```
    (This command will open rviz and will publish all the measurements of the lidar as ROS2 messages).

-----

## 6\) HOW TO RECORD A BAG

To record data into a ROS Bag file:

1.  Enter the repository: `/docker-record`
2.  Launch the container: `./start.sh` 
3.  **Outside of the container**, and while being in the repository `/docker-record`, run the command:
    ```bash
    ./record.sh
    ```

-----
