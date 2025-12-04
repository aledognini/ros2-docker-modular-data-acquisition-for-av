--- NODE 1: CAN -> ROS2 decoder --- (can_decoder) NB: it uses can_msgs folder
This node reads data from a vehicle's CAN bus and publishes it to several ROS2 topics.


Command to launch the node: /root/ROS2_ws/install/can_decoder/bin/decoder_node


Main Topics:
/imu/data_raw (sensor_msgs/Imu)
/vehicle/velocity (geometry_msgs/TwistStamped)



Message Structures Detail with all the units of measurement:

sensor_msgs/Imu
    header: Contains timestamp and frame_id.
    angular_velocity:
       x: Roll rate [rad/s]
       y: Pitch rate [rad/s]
       z: Yaw rate [rad/s]
    linear_acceleration:
       x: Longitudinal acceleration [m/s²]
       y: Lateral acceleration [m/s²]
       z: Vertical acceleration [m/s²]

geometry_msgs/TwistStamped
    header: Contains timestamp and frame_id.
    twist.linear:
       x: Forward/backward speed [m/s]
       y: Sideways speed [m/s]
       z: Up/down speed [m/s]
    twist.angular:
       x: Roll rate [rad/s]
       y: Pitch rate [rad/s]
       z: Yaw rate [rad/s]

Full List of Individual Topics:

    /can_bus/acc_x: [g]

    /can_bus/acc_y: [g]

    /can_bus/acc_z: [g]

    /can_bus/bcu_braking_auto_mode: [enumerated (0:OFF, 1:DEBUG, 2:CAR, 3:NUC)]

    /can_bus/bcu_braking_mode: [enumerated (0:OFF, 1:AUTO, 2:DEBUG, 3:P_vs_V, 4:EMERGENCY)]

    /can_bus/bcu_debug_enable: [boolean]

    /can_bus/bcu_error: [enumerated (0:no error, 1-7:see DBC)]

    /can_bus/bcu_motor_curr: [boolean]

    /can_bus/bcu_motor_pwm: [%]

    /can_bus/bcu_pref: [bar]

    /can_bus/bcu_raw_data_vbat: [bit]

    /can_bus/bcu_raw_data_mo_tcurr: [bit]

    /can_bus/bcu_raw_data_psf: [bit]

    /can_bus/bcu_raw_data_psr: [bit]

    /can_bus/bcu_raw_data_sh_fcurr: [bit]

    /can_bus/bcu_raw_data_sh_rcurr: [bit]

    /can_bus/bcu_raw_data_tc_fcurr: [bit]

    /can_bus/bcu_raw_data_tc_rcurr: [bit]

    /can_bus/bcu_ref_brake_pressure_ccu: [bar]

    /can_bus/bcu_ref_brake_pressure_nuc: [bar]

    /can_bus/bcu_speed_enable: [boolean]

    /can_bus/bcu_counter_mode: [s]

    /can_bus/bcu_red_flag: [boolean]

    /can_bus/bcu_sh_front_curr: [boolean]

    /can_bus/bcu_sh_front_pwm: [%]

    /can_bus/bcu_sh_rear_curr: [boolean]

    /can_bus/bcu_sh_rear_pwm: [%]

    /can_bus/bcu_tc_front_curr: [boolean]

    /can_bus/bcu_tc_front_pwm: [%]

    /can_bus/bcu_tc_rear_curr: [boolean]

    /can_bus/bcu_tc_rear_pwm: [%]

    /can_bus/bdc_pref_debug: [bar]

    /can_bus/car_speed: [km/h]

    /can_bus/ccu_brake_enable: [boolean]

    /can_bus/ccu_car_speed: [km/h]

    /can_bus/ccu_cruise_i: [none]

    /can_bus/ccu_cruise_p: [none]

    /can_bus/ccu_doors: [boolean]

    /can_bus/ccu_drive_neutral_reverse: [enumerated (0:Neutral, 1:Reverse, 2:Forward, 3:Emergency)]

    /can_bus/ccu_error: [enumerated (0:no error, 1-5:see DBC)]

    /can_bus/ccu_gain_d: [none]

    /can_bus/ccu_gain_i: [none]

    /can_bus/ccu_gain_k_brake: [none]

    /can_bus/ccu_gain_k_gas: [none]

    /can_bus/ccu_gain_p: [none]

    /can_bus/ccu_horn: [boolean]

    /can_bus/ccu_key: [boolean]

    /can_bus/ccu_lights: [boolean]

    /can_bus/ccu_party_enable: [boolean]

    /can_bus/ccu_ref_brake_pressure: [bar]

    /can_bus/ccu_speed_enable: [boolean]

    /can_bus/ccu_status: [enumerated (0:idle, 1:starting, 2:on, 3:party, 4:emergency)]

    /can_bus/ccu_throttle: [%]

    /can_bus/ccu_throttle_enable: [boolean]

    /can_bus/ccu_turning_light_l: [boolean]

    /can_bus/ccu_turning_light_r: [boolean]

    /can_bus/debug_bcu_braking_mode: [enumerated (0-4)]

    /can_bus/debug_bcu_debug_enable: [boolean]

    /can_bus/debug_bcu_pref: [bar]

    /can_bus/debug_bcu_mo_tref: [%]

    /can_bus/debug_bcu_sh_fref: [%]

    /can_bus/debug_bcu_sh_rref: [%]

    /can_bus/debug_bcu_tc_fref: [%]

    /can_bus/debug_bcu_tc_rref: [%]

    /can_bus/ref_ax: [m/s^2]

    /can_bus/ref_brake_pressure: [bar]

    /can_bus/ref_brakes_enable: [boolean]

    /can_bus/ref_car_speed: [km/h]

    /can_bus/ref_doors: [boolean]

    /can_bus/ref_drive_neutral_reverse: [enumerated (0-3)]

    /can_bus/ref_horn: [boolean]

    /can_bus/ref_key: [boolean]

    /can_bus/ref_lights: [boolean]

    /can_bus/ref_party_mode: [boolean]

    /can_bus/ref_speed_control_enable: [boolean]

    /can_bus/ref_steering_wheel_angle: [deg]

    /can_bus/ref_steering_wheel_enable: [boolean]

    /can_bus/ref_throttle: [%]

    /can_bus/ref_throttle_enable: [boolean]

    /can_bus/ref_turning_light_l: [boolean]

    /can_bus/ref_turning_light_r: [boolean]

    /can_bus/reserved1...7: [none]

    /can_bus/roll_rate: [deg/s]

    /can_bus/scu_battery_voltage: [V]

    /can_bus/scu_steering_wheel_angle: [deg]

    /can_bus/scu_steering_wheel_error: [enumerated (0-4)]

    /can_bus/scu_steering_wheel_motor_current: [A]

    /can_bus/scu_steering_wheel_motor_pwm: [%]

    /can_bus/scu_steering_wheel_status: [enumerated (0-2)]

    /can_bus/speed_sensor_db_pulses_fl...rr: [none]

    /can_bus/ss_counter: [none]

    /can_bus/ss_steering_angle: [deg]

    /can_bus/ss_steering_speed: [deg/s]

    /can_bus/ss_unknown: [none]

    /can_bus/unused1...3: [none]

    /can_bus/wheel_speed_fl...rr: [km/h]

    /can_bus/yaw_rate: [deg/s]
