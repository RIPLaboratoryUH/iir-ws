import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

import socket
import struct
import time

CAN_FRAME_FORMAT = '=IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF

ADDR = {
        'priority': 0x18,
        'bms': 0x01,
        'pc': 0x40
}

DATA_IDS = {
    'soc': 0x90,
    'min_max_voltage': 0x91,
    'min_max_temp': 0x92,
}

class DalyBMSNode(Node):

    def __init__(self):
        super().__init__('daly_bms_node')
        
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('poll_period', 1.0)
        self.declare_parameter('battery_topic', '/battery/telemetry')
        self.declare_parameter('num_cells', 7)

        self.can_interface = (
                self.get_parameter('can_interface')
                .get_parameter_value()
                .string_value
        )

        self.poll_period = (
                self.get_parameter('poll_period')
                .get_parameter_value()
                .double_value
        )

        self.battery_topic = (
                self.get_parameter('battery_topic')
                .get_parameter_value()
                .string_value
        )

        self.num_cells = (
                self.get_parameter('num_cells')
                .get_parameter_value()
                .integer_value
        )

        # Create socket for CAN0 communication

        self.can_socket = socket.socket(
                socket.PF_CAN,
                socket.SOCK_RAW,
                socket.CAN_RAW
        )

        self.can_socket.bind((self.can_interface,))
        self.can_socket.settimeout(0.1)

        self.get_logger().info('Initialized CAN interface')

        self.publisher = self.create_publisher(
                BatteryState,
                self.battery_topic,
                10
        )

        self.voltage = None
        self.current = None
        self.soc     = None
        self.cell_voltages = []
        self.temperatures = []

        self.timer = self.create_timer(
                self.poll_period,
                self.timer_callback
        )

    #def timer_callback(self):

    def build_request(self, data_name):
        data_id = DATA_IDS[data_name]

        can_id = ( ADDR['priority'] << 24 ) | ( data_id << 16 ) | ( ADDR['bms'] << 8) | ( ADDR['pc'] )
        socketcan_id = can_id | CAN_EFF_FLAG

        payload = bytes([0x00] * 8)
        data_length = len(payload)

        frame = struct.pack(
                CAN_FRAME_FORMAT,
                socketcan_id,
                data_length,
                payload
        )

        return frame


    def send_request(self, data_name):
        frame = self.build_request(data_name)
        self.can_socket.send(frame)

    def receive_frame(self):
        try:
            raw_frame = self.can_socket.recv(CAN_FRAME_SIZE)

            socketcan_id, data_length, payload = struct.unpack(
                    CAN_FRAME_FORMAT,
                    raw_frame
            )

            return socketcan_id, payload[:data_length]

        except socket.timeout:
            return None

    def validate_frame(self, frame, data_name):
        data_id = DATA_IDS[data_name]
        can_id = frame & CAN_EFF_MASK

        expected_id = ( ADDR['priority'] << 24 ) | ( data_id <<  16 ) | ( ADDR['pc'] << 8 ) | ( ADDR['bms'] )

        return can_id == expected_id

    def decode_0x90(self, payload):
        voltage = int.from_bytes(payload[0:2], 'big') / 10.0
        gather_voltage = int.from_bytes(payload[2:4], 'big') / 10.0
        current = int.from_bytes(payload[4:6], 'big') - 30000.0
        soc = int.from_bytes(payload[6:8], 'big') / 10.0

        return voltage, gather_voltage, current, soc

    def closeSocket(self):
        self.can_socket.close()

    def timer_callback(self):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        self.send_request('soc')

        deadline = time.monotonic() + 1.0
        response_received = False

        while time.monotonic() < deadline:
            result = self.receive_frame()

            if result is None:
                break

            can_id, payload = result

            if self.validate_frame(can_id, 'soc'):
                voltage, gather_voltage, current, soc = self.decode_0x90(payload)
                response_received = True

                msg.voltage = voltage
                msg.current = float(current)
                msg.percentage = soc / 100.0

                self.publisher.publish(msg)
                self.get_logger().info("Message published")
                break

        if not response_received:
            self.get_logger().info("No DALY BMS response")

def main(args=None):
    rclpy.init(args=args)

    daly_bms_node = DalyBMSNode()

    rclpy.spin(daly_bms_node)

    daly_bms_node.destroy_node()
    daly_bms_node.closeSocket()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
