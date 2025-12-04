import rclpy
from rclpy.node import Node
import cantools
import can
from std_msgs.msg import Float64, Bool, UInt8, Int64 # MODIFICA: Aggiunto Int64

# L'ID del messaggio di comando, dal file DBC.
CAR_CONTROL_COMMAND_ID = 257

class CanCommanderNode(Node):
    """
    Questo nodo aggrega i comandi da vari topic ROS (/cmd/...) e
    lo stato reale del veicolo dai topic /can_bus/... (pubblicati 
    dal decoder_node).
    
    Invia un singolo messaggio CAN di comando a 100Hz, dando priorità
    ai comandi /cmd/... rispetto allo stato /can_bus/... una volta
    che un comando è stato ricevuto.
    """
    def __init__(self):
        super().__init__('can_commander_node')
        self.get_logger().info('CAN Commander Node (v3: ROS-Native) has started.')

        # --- Parametri ---
        dbc_file_path = '/root/data/ZD1_plus.dbc'
        can_interface = 'can2'
        self.cmd_topic_prefix = '/cmd'
        
        # MODIFICA: Aggiunto il prefisso per i topic di stato
        self.state_topic_prefix = '/can_bus' 
        
        self.publish_frequency = 100.0

        # --- Setup DBC e CAN Bus (solo per invio) ---
        self.db = cantools.database.load_file(dbc_file_path)
        self.bus = can.interface.Bus(channel=can_interface, bustype='socketcan')
        self.get_logger().info(f"DBC loaded and connected to CAN interface '{can_interface}' (send only).")

        # --- Contenitore dello stato per il messaggio di comando ---
        # Inizializzato con valori sicuri e predefiniti.
        self.command_data = {
            'Ref_SteeringWheelAngle': 0.0,
            'Ref_SteeringWheelEnable': False,
            'Ref_Throttle': 0.0,
            'Ref_ThrottleEnable': False,
            'Ref_BrakePressure': 0.0,
            'Ref_BrakesEnable': False,
            'Ref_CarSpeed': 0.0,
            'Ref_SpeedControlEnable': False,
            'Ref_Ax': 0.0,
            'Ref_TurningLightL': False,
            'Ref_TurningLightR': False,
            'Ref_Lights': False,
            'Ref_Horn': False,
            'Ref_Key': False,
            'Ref_Doors': False,
            'Ref_PartyMode': False,
            'Ref_DriveNeutralReverse': 2
        }

        # MODIFICA: Dizionario per tracciare i comandi ricevuti
        # Questo è il cuore della logica di "shadowing".
        self.has_received_command = {key: False for key in self.command_data}

        # --- Creazione Subscriber ---
        self.get_logger().info("Creating COMMAND subscribers (/cmd)...")
        self.create_command_subscribers()
        
        self.get_logger().info("Creating STATE subscribers (/can_bus) for shadowing...")
        self.create_state_subscribers()

        # --- Timer per l'invio a 100Hz ---
        timer_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"CAN command publisher timer created with {self.publish_frequency}Hz frequency.")

    def create_command_subscribers(self):
        """Crea i subscriber per i topic di COMANDO."""
        # Comandi di guida
        self.create_subscription(Float64, f"{self.cmd_topic_prefix}/ref_steering_wheel_angle", self.cmd_steering_angle_callback, 10)
        self.create_subscription(Float64, f"{self.cmd_topic_prefix}/ref_throttle", self.cmd_throttle_callback, 10)
        self.create_subscription(Float64, f"{self.cmd_topic_prefix}/ref_brake_pressure", self.cmd_brake_pressure_callback, 10)
        self.create_subscription(Float64, f"{self.cmd_topic_prefix}/ref_car_speed", self.cmd_car_speed_callback, 10)
        self.create_subscription(Float64, f"{self.cmd_topic_prefix}/ref_ax", self.cmd_ax_callback, 10)
        
        # Flag di abilitazione
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_steering_wheel_enable", self.cmd_steering_enable_callback, 10)
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_throttle_enable", self.cmd_throttle_enable_callback, 10)
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_brakes_enable", self.cmd_brakes_enable_callback, 10)
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_speed_control_enable", self.cmd_speed_control_enable_callback, 10)
        
        # Comandi ausiliari
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_turning_light_l", self.cmd_turn_light_l_callback, 10)
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_turning_light_r", self.cmd_turn_light_r_callback, 10)
        self.create_subscription(UInt8, f"{self.cmd_topic_prefix}/ref_drive_neutral_reverse", self.cmd_gear_callback, 10)
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_lights", self.cmd_lights_callback, 10)
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_horn", self.cmd_horn_callback, 10)
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_key", self.cmd_key_callback, 10)
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_doors", self.cmd_doors_callback, 10)
        self.create_subscription(Bool, f"{self.cmd_topic_prefix}/ref_party_mode", self.cmd_party_mode_callback, 10)

    def create_state_subscribers(self):
        """
        Crea i subscriber per i topic di STATO (dal decoder_node)
        per inizializzare e "shadoware" lo stato del veicolo.
        I nomi dei topic derivano da decoder_node.py (es. CCU_Lights -> ccu_lights)
        """
        # Mappatura dai messaggi di stato (ID 528, 517, 784) ai nostri comandi
        # Nota: i tipi di messaggio (Bool, Float64, Int64) devono corrispondere
        # a quelli pubblicati dal decoder_node.py
        
        # Da CarControlUnitStatus (ID 528)
        self.create_subscription(Int64, f"{self.state_topic_prefix}/ccu_throttle", self.state_throttle_callback, 10)
        self.create_subscription(Bool, f"{self.state_topic_prefix}/ccu_turning_light_r", self.state_turn_light_r_callback, 10)
        self.create_subscription(Bool, f"{self.state_topic_prefix}/ccu_turning_light_l", self.state_turn_light_l_callback, 10)
        self.create_subscription(Bool, f"{self.state_topic_prefix}/ccu_lights", self.state_lights_callback, 10)
        self.create_subscription(Bool, f"{self.state_topic_prefix}/ccu_key", self.state_key_callback, 10)
        self.create_subscription(Bool, f"{self.state_topic_prefix}/ccu_horn", self.state_horn_callback, 10)
        self.create_subscription(Bool, f"{self.state_topic_prefix}/ccu_doors", self.state_doors_callback, 10)
        self.create_subscription(Int64, f"{self.state_topic_prefix}/ccu_drive_neutral_reverse", self.state_gear_callback, 10)
        self.create_subscription(Int64, f"{self.state_topic_prefix}/ccu_ref_brake_pressure", self.state_brake_pressure_callback, 10)
        
        # Da CarSpeedsSensor (ID 517)
        self.create_subscription(Float64, f"{self.state_topic_prefix}/car_speed", self.state_car_speed_callback, 10)
        
        # Da SteerControlUnitStatus (ID 784)
        self.create_subscription(Float64, f"{self.state_topic_prefix}/scu_steering_wheel_angle", self.state_steering_angle_callback, 10)

    # --- Callback del Timer ---
    def timer_callback(self):
        """Chiamato a 100Hz, invia lo stato corrente."""
        self.send_can_command()

    # --- Funzioni di Callback di COMANDO (/cmd/...) ---
    # Queste callback impostano il valore E il flag "has_received_command".
    
    def cmd_steering_angle_callback(self, msg):
        self.command_data['Ref_SteeringWheelAngle'] = msg.data
        self.has_received_command['Ref_SteeringWheelAngle'] = True

    def cmd_throttle_callback(self, msg):
        self.command_data['Ref_Throttle'] = msg.data
        self.has_received_command['Ref_Throttle'] = True

    def cmd_brake_pressure_callback(self, msg):
        self.command_data['Ref_BrakePressure'] = msg.data
        self.has_received_command['Ref_BrakePressure'] = True

    def cmd_car_speed_callback(self, msg):
        self.command_data['Ref_CarSpeed'] = msg.data
        self.has_received_command['Ref_CarSpeed'] = True
        
    def cmd_ax_callback(self, msg):
        self.command_data['Ref_Ax'] = msg.data
        self.has_received_command['Ref_Ax'] = True

    def cmd_steering_enable_callback(self, msg):
        self.command_data['Ref_SteeringWheelEnable'] = msg.data
        self.has_received_command['Ref_SteeringWheelEnable'] = True
        
    def cmd_throttle_enable_callback(self, msg):
        self.command_data['Ref_ThrottleEnable'] = msg.data
        self.has_received_command['Ref_ThrottleEnable'] = True

    def cmd_brakes_enable_callback(self, msg):
        self.command_data['Ref_BrakesEnable'] = msg.data
        self.has_received_command['Ref_BrakesEnable'] = True
        
    def cmd_speed_control_enable_callback(self, msg):
        self.command_data['Ref_SpeedControlEnable'] = msg.data
        self.has_received_command['Ref_SpeedControlEnable'] = True

    def cmd_turn_light_l_callback(self, msg):
        self.command_data['Ref_TurningLightL'] = msg.data
        self.has_received_command['Ref_TurningLightL'] = True

    def cmd_turn_light_r_callback(self, msg):
        self.command_data['Ref_TurningLightR'] = msg.data
        self.has_received_command['Ref_TurningLightR'] = True
        
    def cmd_gear_callback(self, msg):
        self.command_data['Ref_DriveNeutralReverse'] = msg.data
        self.has_received_command['Ref_DriveNeutralReverse'] = True

    def cmd_lights_callback(self, msg):
        self.command_data['Ref_Lights'] = msg.data
        self.has_received_command['Ref_Lights'] = True

    def cmd_horn_callback(self, msg):
        self.command_data['Ref_Horn'] = msg.data
        self.has_received_command['Ref_Horn'] = True

    def cmd_key_callback(self, msg):
        self.command_data['Ref_Key'] = msg.data
        self.has_received_command['Ref_Key'] = True

    def cmd_doors_callback(self, msg):
        self.command_data['Ref_Doors'] = msg.data
        self.has_received_command['Ref_Doors'] = True

    def cmd_party_mode_callback(self, msg):
        self.command_data['Ref_PartyMode'] = msg.data
        self.has_received_command['Ref_PartyMode'] = True

    # --- Funzioni di Callback di STATO (/can_bus/...) ---
    # Queste callback aggiornano il valore SOLO SE un comando
    # non è ancora stato ricevuto per quel segnale.
    
    def state_steering_angle_callback(self, msg):
        if not self.has_received_command['Ref_SteeringWheelAngle']:
            self.command_data['Ref_SteeringWheelAngle'] = msg.data

    def state_throttle_callback(self, msg):
        if not self.has_received_command['Ref_Throttle']:
            self.command_data['Ref_Throttle'] = float(msg.data) # Converte Int64 in float se necessario

    def state_brake_pressure_callback(self, msg):
        if not self.has_received_command['Ref_BrakePressure']:
            self.command_data['Ref_BrakePressure'] = float(msg.data)

    def state_car_speed_callback(self, msg):
        if not self.has_received_command['Ref_CarSpeed']:
            self.command_data['Ref_CarSpeed'] = msg.data

    def state_turn_light_l_callback(self, msg):
        if not self.has_received_command['Ref_TurningLightL']:
            self.command_data['Ref_TurningLightL'] = msg.data

    def state_turn_light_r_callback(self, msg):
        if not self.has_received_command['Ref_TurningLightR']:
            self.command_data['Ref_TurningLightR'] = msg.data

    def state_gear_callback(self, msg):
        if not self.has_received_command['Ref_DriveNeutralReverse']:
            self.command_data['Ref_DriveNeutralReverse'] = msg.data

    def state_lights_callback(self, msg):
        if not self.has_received_command['Ref_Lights']:
            self.command_data['Ref_Lights'] = msg.data

    def state_horn_callback(self, msg):
        if not self.has_received_command['Ref_Horn']:
            self.command_data['Ref_Horn'] = msg.data

    def state_key_callback(self, msg):
        if not self.has_received_command['Ref_Key']:
            self.command_data['Ref_Key'] = msg.data

    def state_doors_callback(self, msg):
        if not self.has_received_command['Ref_Doors']:
            self.command_data['Ref_Doors'] = msg.data
            
    # --- Funzione di Invio ---

    def send_can_command(self):
        """Codifica e invia lo stato corrente (aggregato) come messaggio CAN."""
        
        # Nota: in un executor ROS2 single-threaded, un lock non è 
        # strettamente necessario. Per sicurezza in caso di esecutori
        # multi-threaded, copiare i dati è una buona pratica.
        local_data_copy = self.command_data.copy()
        
        try:
            encoded_data = self.db.encode_message('CarControlCommand', local_data_copy)
            can_message = can.Message(
                arbitration_id=CAR_CONTROL_COMMAND_ID,
                data=encoded_data,
                is_extended_id=False
            )
            self.bus.send(can_message)
            self.get_logger().debug(f"Sent CAN command (100Hz).")
            
        except Exception as e:
            self.get_logger().error(f"Failed to encode or send CAN message: {e}")
            self.get_logger().error(f"Data that failed: {local_data_copy}")

def main(args=None):
    rclpy.init(args=args)
    can_commander_node = CanCommanderNode()
    try:
        rclpy.spin(can_commander_node)
    except KeyboardInterrupt:
        pass
    finally:
        can_commander_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
