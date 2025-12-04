import rclpy
from rclpy.node import Node
import cantools
import can
import re
import math

# Import standard message types
from std_msgs.msg import Float64, Int64, Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

# NOTE: The direct import of NamedSignalValue is removed to avoid ModuleNotFoundError.
# We will check for the type by its name instead.

# --- Constants for Unit Conversion ---
G_TO_MS2 = 9.80665  # Standard gravity to m/s^2
DEG_TO_RAD = math.pi / 180.0
KMH_TO_MS = 1.0 / 3.6

# La funzione to_snake_case è stata rimossa come richiesto.

class CanDecoderNode(Node):
    """
    This node decodes CAN messages and publishes data on multiple topics:
    1. Individual signals on their own topics (hardcoded from a map).
    2. Standard ROS2 messages for interoperability (/imu/data_can, /vehicle/velocity).
    """
    def __init__(self):
        super().__init__('can_decoder_node')
        self.get_logger().info('Standard CAN Decoder Node has started.')

        # --- Parameters ---
        dbc_file_path = '/root/data/ZD1_plus.dbc'
        can_interface = 'can2'
        # self.topic_prefix non è più necessario poiché i topic sono hardcoded

        # --- DBC and CAN Bus Setup ---
        try:
            self.db = cantools.database.load_file(dbc_file_path)
        except FileNotFoundError:
            self.get_logger().error(f"DBC file not found at '{dbc_file_path}'. Shutting down.")
            rclpy.shutdown()
            return

        # --- Publisher Creation ---
        self.setup_publishers()

        # --- Message State Holders ---
        self.imu_msg = Imu()
        self.twist_msg = TwistStamped()
        self.imu_msg.header.frame_id = 'imu_link'
        self.twist_msg.header.frame_id = 'base_link'

        # --- Start CAN Bus Listener ---
        try:
            self.bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
            self.get_logger().info(f"Listening on CAN interface '{can_interface}'.")
        except Exception as e:
            self.get_logger().error(f"Could not start CAN bus: {e}")
            rclpy.shutdown()
            return
            
        # --- MODIFICA: Timer impostato a 500Hz (0.002s) ---
        self.timer = self.create_timer(0.002, self.read_can_messages)

    def _get_signal_topic_map(self):
        """
        Restituisce la mappa hardcoded dei segnali DBC ai topic ROS.
        Questo sostituisce la funzione to_snake_case.
        """
        return {
            # BO_ 529 CarControlGain
            'CCU_Gain_kBrake': '/can_bus/ccu__gain_k_brake',
            'CCU_Gain_I': '/can_bus/ccu__gain_i',
            'CCU_Gain_D': '/can_bus/ccu__gain_d',
            'CCU_Gain_P': '/can_bus/ccu__gain_p',
            'CCU_Gain_kGas': '/can_bus/ccu__gain_k_gas',
            # BO_ 275 BrakeDebugReplace
            'BDC_PrefDebug': '/can_bus/bdc__pref_debug',
            'BCU_Pref': '/can_bus/bcu__pref',
            'BCU_DebugEnable': '/can_bus/bcu__debug_enable',
            'BCU_counterMode': '/can_bus/bcu_counter_mode',
            'BCU_red_flag': '/can_bus/bcu_red_flag',
            'BCU_BrakingAutoMode': '/can_bus/bcu__braking_auto_mode',
            'BCU_BrakingMode': '/can_bus/bcu__braking_mode',
            # BO_ 258 DebugBrakeControlUnit
            'Debug_BCU_MOTref': '/can_bus/debug_bcu_mo_tref',
            'Debug_BCU_TCFref': '/can_bus/debug_bcu_tc_fref',
            'Debug_BCU_TCRref': '/can_bus/debug_bcu_tc_rref',
            'Debug_BCU_SHFref': '/can_bus/debug_bcu_sh_fref',
            'Debug_BCU_SHRref': '/can_bus/debug_bcu_sh_rref',
            'Debug_BCU_Pref': '/can_bus/debug_bcu__pref',
            'Debug_BCU_BrakingMode': '/can_bus/debug_bcu__braking_mode',
            'Debug_BCU_DebugEnable': '/can_bus/debug_bcu__debug_enable',
            # BO_ 274 BrakeRawData2
            'BCU_RawData_TCRcurr': '/can_bus/bcu__raw_data_tc_rcurr',
            'BCU_RawData_TCFcurr': '/can_bus/bcu__raw_data_tc_fcurr',
            'BCU_RawData_SHRcurr': '/can_bus/bcu__raw_data_sh_rcurr',
            'BCU_RawData_SHFcurr': '/can_bus/bcu__raw_data_sh_fcurr',
            # BO_ 273 BrakeRawData1
            'BCU_RawData_Vbat': '/can_bus/bcu__raw_data__vbat',
            'BCU_RawData_PSR': '/can_bus/bcu__raw_data_psr',
            'BCU_RawData_PSF': '/can_bus/bcu__raw_data_psf',
            'BCU_RawData_MOTcurr': '/can_bus/bcu__raw_data_mo_tcurr',
            # BO_ 518 CarSpeedSensorDB
            'SpeedSensor_db_PulsesRR': '/can_bus/speed_sensor_db__pulses_rr',
            'SpeedSensor_db_PulsesRL': '/can_bus/speed_sensor_db__pulses_rl',
            'SpeedSensor_db_PulsesFR': '/can_bus/speed_sensor_db__pulses_fr',
            'SpeedSensor_db_PulsesFL': '/can_bus/speed_sensor_db__pulses_fl',
            # BO_ 272 BrakeControlStatus
            'BCU_TCRearCurr': '/can_bus/bcu_tc_rear_curr',
            'BCU_TCFrontCurr': '/can_bus/bcu_tc_front_curr',
            'BCU_SHRearCurr': '/can_bus/bcu_sh_rear_curr',
            'BCU_MotorCurr': '/can_bus/bcu__motor_curr',
            'BCU_SHFrontCurr': '/can_bus/bcu_sh_front_curr',
            'BCU_Error': '/can_bus/bcu__error',
            'BCU_TCRearPWM': '/can_bus/bcu_tc_rear_pwm',
            'BCU_TCFrontPWM': '/can_bus/bcu_tc_front_pwm',
            'BCU_SpeedEnable': '/can_bus/bcu__speed_enable',
            'BCU_SHRearPWM': '/can_bus/bcu_sh_rear_pwm',
            'BCU_SHFrontPWM': '/can_bus/bcu_sh_front_pwm',
            'BCU_Ref_BrakePressure_NUC': '/can_bus/bcu__ref__brake_pressure_nuc',
            'BCU_Ref_BrakePressure_CCU': '/can_bus/bcu__ref__brake_pressure_ccu',
            'BCU_MotorPWM': '/can_bus/bcu__motor_pwm',
            # BO_ 528 CarControlUnitStatus
            'CCU_Status': '/can_bus/ccu__status',
            'CCU_Throttle': '/can_bus/ccu__throttle',
            'CCU_CarSpeed': '/can_bus/ccu__car_speed',
            'CCU_TurningLightR': '/can_bus/ccu__turning_light_r',
            'CCU_TurningLightL': '/can_bus/ccu__turning_light_l',
            'CCU_Lights': '/can_bus/ccu__lights',
            'CCU_Key': '/can_bus/ccu__key',
            'CCU_Horn': '/can_bus/ccu__horn',
            'CCU_Doors': '/can_bus/ccu__doors',
            'CCU_DriveNeutralReverse': '/can_bus/ccu__drive_neutral_reverse',
            'CCU_ThrottleEnable': '/can_bus/ccu__throttle_enable',
            'CCU_SpeedEnable': '/can_bus/ccu__speed_enable',
            'CCU_Ref_BrakePressure': '/can_bus/ccu__ref__brake_pressure',
            'CCU_PartyEnable': '/can_bus/ccu__party_enable',
            'CCU_Error': '/can_bus/ccu__error',
            'CCU_BrakeEnable': '/can_bus/ccu__brake_enable',
            'CCU_Cruise_P': '/can_bus/ccu__cruise_p',
            'CCU_Cruise_I': '/can_bus/ccu__cruise_i',
            # BO_ 517 CarSpeedsSensor
            'WheelSpeedRR': '/can_bus/wheel_speed_rr',
            'WheelSpeedRL': '/can_bus/wheel_speed_rl',
            'WheelSpeedFR': '/can_bus/wheel_speed_fr',
            'WheelSpeedFL': '/can_bus/wheel_speed_fl',
            'CarSpeed': '/can_bus/car_speed',
            # BO_ 773 SteeringWheelSensor
            'SS_Unknown': '/can_bus/ss__unknown',
            'SS_SteeringSpeed': '/can_bus/ss__steering_speed',
            'SS_Counter': '/can_bus/ss__counter',
            'SS_SteeringAngle': '/can_bus/ss__steering_angle',
            # BO_ 784 SteerControlUnitStatus
            'SCU_SteeringWheelAngle': '/can_bus/scu__steering_wheel_angle',
            'SCU_SteeringWheelStatus': '/can_bus/scu__steering_wheel_status',
            'SCU_SteeringWheelMotorPWM': '/can_bus/scu__steering_wheel_motor_pwm',
            'SCU_SteeringWheelMotorCurrent': '/can_bus/scu__steering_wheel_motor_current',
            'SCU_SteeringWheelError': '/can_bus/scu__steering_wheel_error',
            'SCU_BatteryVoltage': '/can_bus/scu__battery_voltage',
            # BO_ 257 CarControlCommand
            'Ref_PartyMode': '/can_bus/ref__party_mode',
            'Ref_Lights': '/can_bus/ref__lights',
            'Ref_Doors': '/can_bus/ref__doors',
            'Ref_Key': '/can_bus/ref__key',
            'Ref_Horn': '/can_bus/ref__horn',
            'Ref_Ax': '/can_bus/ref__ax',
            'Ref_TurningLightR': '/can_bus/ref__turning_light_r',
            'Ref_ThrottleEnable': '/can_bus/ref__throttle_enable',
            'Ref_BrakesEnable': '/can_bus/ref__brakes_enable',
            'Ref_BrakePressure': '/can_bus/ref__brake_pressure',
            'Ref_Throttle': '/can_bus/ref__throttle',
            'Ref_CarSpeed': '/can_bus/ref__car_speed',
            'Ref_TurningLightL': '/can_bus/ref__turning_light_l',
            'Ref_SpeedControlEnable': '/can_bus/ref__speed_control_enable',
            'Ref_DriveNeutralReverse': '/can_bus/ref__drive_neutral_reverse',
            'Ref_SteeringWheelEnable': '/can_bus/ref__steering_wheel_enable',
            'Ref_SteeringWheelAngle': '/can_bus/ref__steering_wheel_angle',
            # BO_ 372 IMU_0x174
            'YawRate': '/can_bus/yaw_rate',
            'Reserved1': '/can_bus/reserved1',
            'AccY': '/can_bus/acc_y',
            'Reserved2': '/can_bus/reserved2',
            'Unused1': '/can_bus/unused1',
            # BO_ 376 IMU_0x178
            'RollRate': '/can_bus/roll_rate',
            'Reserved3': '/can_bus/reserved3',
            'AccX': '/can_bus/acc_x',
            'Reserved4': '/can_bus/reserved4',
            'Unused2': '/can_bus/unused2',
            # BO_ 380 IMU_0x17C
            'Reserved5': '/can_bus/reserved5',
            'Reserved6': '/can_bus/reserved6',
            'AccZ': '/can_bus/acc_z',
            'Reserved7': '/can_bus/reserved7',
            'Unused3': '/can_bus/unused3',
        }


    def setup_publishers(self):
        """Initializes all required publishers."""
        # 1. Standard message publishers
        self.imu_publisher_ = self.create_publisher(Imu, '/imu/data_can', 10)
        self.velocity_publisher_ = self.create_publisher(TwistStamped, '/vehicle/velocity', 10)

        # 2. Individual signal publishers (MODIFICATO)
        self.individual_publishers_ = {}
        signal_topic_map = self._get_signal_topic_map()

        for message in self.db.messages:
            for signal in message.signals:
                signal_name = signal.name

                # Se il segnale DBC non è nella mappa, saltalo
                if signal_name not in signal_topic_map:
                    # self.get_logger().warn(f"Signal '{signal_name}' from DBC not in topic map. Skipping.")
                    continue
                
                # Ottieni il nome del topic dalla mappa
                topic_name = signal_topic_map[signal_name]
                
                # Determina il tipo di messaggio (logica invariata)
                if signal.length == 1:
                    msg_type = Bool
                elif signal.scale != int(signal.scale):
                    msg_type = Float64
                else:
                    msg_type = Int64
                
                publisher = self.create_publisher(msg_type, topic_name, 10)
                self.individual_publishers_[signal_name] = {'pub': publisher, 'type': msg_type}
        
        self.get_logger().info("All publishers created successfully.")

    def read_can_messages(self):
        """Reads a CAN message, decodes it, and publishes the data."""
        can_msg = self.bus.recv(timeout=0.0) # Il timeout è 0.0, il timer gestisce la frequenza
        if can_msg is None:
            return

        try:
            decoded_signals = self.db.decode_message(can_msg.arbitration_id, can_msg.data)
            
            for signal_name, signal_value in decoded_signals.items():
                # CORRECTED: Check the type by its name to avoid import errors.
                numeric_value = signal_value.value if type(signal_value).__name__ == 'NamedSignalValue' else signal_value

                # Action 1: Publish to individual topic
                self.publish_individual_signal(signal_name, numeric_value)
                
                # Action 2: Populate standard ROS2 messages
                self.populate_standard_messages(signal_name, numeric_value)
            
            # Publish standard messages (they are updated incrementally)
            now = self.get_clock().now().to_msg()
            self.imu_msg.header.stamp = now
            self.twist_msg.header.stamp = now
            
            self.imu_publisher_.publish(self.imu_msg)
            self.velocity_publisher_.publish(self.twist_msg)

        except KeyError:
            # Ignora i messaggi CAN non presenti nel DBC
            pass
        except Exception as e:
            self.get_logger().warn(f"An error occurred during message processing: {e}")

    def publish_individual_signal(self, signal_name, value):
        """Publishes a single decoded signal to its specific topic."""
        # Questa funzione si basa sulla mappa self.individual_publishers_
        # creata in setup_publishers, quindi funziona correttamente
        # con la nuova logica
        if signal_name in self.individual_publishers_:
            pub_info = self.individual_publishers_[signal_name]
            publisher = pub_info['pub']
            msg_type = pub_info['type']
            
            individual_msg = msg_type()
            if msg_type == Bool: individual_msg.data = bool(value)
            elif msg_type == Int64: individual_msg.data = int(value)
            else: individual_msg.data = float(value)
            
            publisher.publish(individual_msg)

    def populate_standard_messages(self, signal_name, value):
        """Populates the fields of standard ROS2 messages, with unit conversions."""
        # Logica invariata, basata sui nomi dei segnali DBC
        
        # IMU Message
        if signal_name == 'AccX': self.imu_msg.linear_acceleration.x = float(value) * G_TO_MS2
        elif signal_name == 'AccY': self.imu_msg.linear_acceleration.y = float(value) * G_TO_MS2
        elif signal_name == 'AccZ': self.imu_msg.linear_acceleration.z = float(value) * G_TO_MS2
        elif signal_name == 'RollRate': self.imu_msg.angular_velocity.x = float(value) * DEG_TO_RAD
        elif signal_name == 'YawRate': self.imu_msg.angular_velocity.z = float(value) * DEG_TO_RAD
        
        # Velocity (Twist) Message
        if signal_name == 'CarSpeed': self.twist_msg.twist.linear.x = float(value) * KMH_TO_MS
        elif signal_name == 'YawRate': self.twist_msg.twist.angular.z = float(value) * DEG_TO_RAD

def main(args=None):
    rclpy.init(args=args)
    can_decoder_node = CanDecoderNode()
    try:
        rclpy.spin(can_decoder_node)
    except KeyboardInterrupt:
        pass
    finally:
        can_decoder_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
