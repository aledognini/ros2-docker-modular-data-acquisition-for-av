---- HOW TO USE EACH SINGLE CONTAINER ----


0)FIRST THING TO DO:
set the correct configurations for CycloneDDS. Launch these commands:
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728
sudo ip link set lo multicast on




1) DUAL-ANTENNA GNSS

1.1) enter in the repository /docker-gnss
1.2) ./start.sh
1.3) inside the gnss_ros2_container launch these commands:
         terminal 1 -> cd /root/scripts && python3 rtk_write.py
         terminal 2 -> ros2 launch gnss_bringup antenna_front.launch.py
         terminal 3 -> ros2 launch gnss_bringup antenna_rear.launch.py
1.4) to check the correct functioning watch the topic list
1.5) in order to get the mqtt messages pubblished online launch this command:
         ros2 launch ros2_mqtt_bridge car.launch.py
         (this command is used to publish the gps position on the MQTT broker)
         cd /root/mqtt_script && python3 mqtt_warning_bridge_node.py
         (this command is used to get the boolean value for the topic /pedestrian_warning from the MQTT broker)




2) CAN-BUS

2.1) outside the container launch these two commands (NB: command 'ip a' to check if can2 is correct):
         command 1 -> sudo ip link set can2 type can bitrate 500000
         command 2 -> sudo ip link set up can2
2.2) enter in the repository /docker-can
2.3) ./start.sh
2.4) inside the can_ros2_container launch these commands:
         (command to check the functioning: 'candump can2')
         terminal 1 -> ros2 run can_decoder decoder_node
         terminal 2 -> ros2 run can_commander commander_node
2.5) to check the correct functioning watch the topic list
2.6) to pubblish commands to command the car you need to pubblish on the topics /cmd/... (NB: the type of message has to be correct)
2.7) to do the tests for the commander node you can launch these 2 nodes:
         test 1 -> ros2 run test1_commander test1_command_node
         test 2 -> ros2 run test2_commander test2_command_node
         (the first test controls just the lights and the horn for an initial very simple test, while the second test controls the motor and the brakes to do a more complex test on the can)
2.8) to visualize the warning signal for pedestrian detection launch this command: 
         ros2 launch pedestrian_visualizer visualizer.launch.py
         (       if you want to check how the marker change color launch these two commands:    )
         (           ros2 topic pub /pedestrian_warning std_msgs/msg/Bool "data: true"          )
         (           ros2 topic pub /pedestrian_warning std_msgs/msg/Bool "data: false"         )
 




3) ZED CAMERA

3.1) enter in the repository /docker-zed
3.2) ./start.sh
3.3) to chech the connection with the cameras launch 'ZED_Explorer' (NB: every time the connection with the zed is changed a reboot is necessary)
3.4) inside the zed_ros2_container launch these commands:
         3.4.1) CASE only one zed camera:
                    ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
         3.4.2) CASE two zed cameras:
                    terminal 1 -> ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx camera_name:=zed_front
                    terminal 2 -> ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx camera_name:=zed_rear     
3.5) to check the correct functioning watch the topic list and use rviz2





4) IMU

4.1) enter in the repository /docker-imu
4.2) ./start.sh
4.3) inside the imu_ros2_container launch these command:
         ros2 launch microstrain_inertial_examples cv7_launch.py





5) LiDAR

5.1) enter in the repository /docker-ouster
5.2) ./start.sh
5.3) inside the ouster_ros_container launch this command:
ros2 launch ouster_ros sensor.independent.launch.xml \
    sensor_hostname:=192.168.1.50 \
    azimuth_window_start:=90000 \
    azimuth_window_end:=270000      

(this command will open rviz and will pubblish all the measurements of the lidar as ROS2 messages)   





6) HOW TO RECORD A BAG

6.1) enter in the repository /docker-record
6.2) ./start.sh
6.3) outside of the container and being in the repository /docker-record run the command: ./record.sh
       
