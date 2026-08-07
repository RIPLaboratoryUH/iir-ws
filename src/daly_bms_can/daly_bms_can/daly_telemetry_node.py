import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

import socket
import struct
import time
import math


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
    'soc':              0x90,
    'min_max_voltage':  0x91,
    'min_max_temp':     0x92,
    'mos_status':       0x93,
    'status_info':      0x94,
    'cell_voltage':     0x95,
    'cell_temperature': 0x96,
    'cell_balance':     0x97,
}


POLL_ORDER = [
    'soc',
    'min_max_voltage',
    'min_max_temp',
    'mos_status',
    'status_info',
    'cell_voltage',
    'cell_temperature',
    'cell_balance',
]


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

        self.can_socket = socket.socket(
            socket.PF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW
        )

        self.can_socket.bind((self.can_interface,))
        self.can_socket.settimeout(0.1)

        self.get_logger().info(
            f'Initialized CAN interface: {self.can_interface}'
        )

        self.publisher = self.create_publisher(
            BatteryState,
            self.battery_topic,
            10
        )

        # 0x90
        self.voltage = None
        self.gather_voltage = None
        self.current = None
        self.soc = None

        # 0x91
        self.max_cell_voltage = None
        self.max_cell_num = None
        self.min_cell_voltage = None
        self.min_cell_num = None

        # 0x92
        self.max_temp = None
        self.max_temp_sensor = None
        self.min_temp = None
        self.min_temp_sensor = None

        # 0x93
        self.bms_state = None
        self.charge_mos = None
        self.discharge_mos = None
        self.cycles = None
        self.remaining_capacity_ah = None

        # 0x94
        self.reported_num_cells = None
        self.num_temperature_sensors = None
        self.charger_connected = None
        self.load_connected = None
        self.digital_inputs = []
        self.digital_outputs = []

        # 0x95
        self.cell_voltages = [math.nan] * self.num_cells

        # 0x96
        self.temperatures = []

        # 0x97
        self.balance_states = [False] * self.num_cells

        self.timer = self.create_timer(
            self.poll_period,
            self.timer_callback
        )

    def build_request(self, data_name):

        data_id = DATA_IDS[data_name]

        can_id = (
            (ADDR['priority'] << 24)
            | (data_id << 16)
            | (ADDR['bms'] << 8)
            | ADDR['pc']
        )

        socketcan_id = can_id | CAN_EFF_FLAG

        payload = bytes([0x00] * 8)

        frame = struct.pack(
            CAN_FRAME_FORMAT,
            socketcan_id,
            len(payload),
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

        expected_id = (
            (ADDR['priority'] << 24)
            | (data_id << 16)
            | (ADDR['pc'] << 8)
            | ADDR['bms']
        )

        return can_id == expected_id

    def request_single_frame(self, data_name, timeout=0.25):

        self.send_request(data_name)

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            result = self.receive_frame()

            if result is None:
                continue

            can_id, payload = result

            if self.validate_frame(can_id, data_name):
                return payload

        return None

    def request_multi_frame(
        self,
        data_name,
        expected_frames,
        timeout=0.4
    ):

        self.send_request(data_name)

        frames = {}

        deadline = time.monotonic() + timeout

        while (
            time.monotonic() < deadline
            and len(frames) < expected_frames
        ):

            result = self.receive_frame()

            if result is None:
                continue

            can_id, payload = result

            if not self.validate_frame(can_id, data_name):
                continue

            if len(payload) == 0:
                continue

            frame_num = payload[0]

            if frame_num == 0xFF:
                continue

            frames[frame_num] = payload

        return frames


    def decode_0x90(self, payload):

        voltage = (
            int.from_bytes(payload[0:2], 'big')
            / 10.0
        )

        gather_voltage = (
            int.from_bytes(payload[2:4], 'big')
            / 10.0
        )

        raw_current = int.from_bytes(
            payload[4:6],
            'big'
        )

        current = (
            raw_current - 30000
        ) / 10.0

        soc = (
            int.from_bytes(payload[6:8], 'big')
            / 10.0
        )

        return (
            voltage,
            gather_voltage,
            current,
            soc
        )

    def decode_0x91(self, payload):

        max_cell_voltage = (
            int.from_bytes(payload[0:2], 'big')
            / 1000.0
        )

        max_cell_num = payload[2]

        min_cell_voltage = (
            int.from_bytes(payload[3:5], 'big')
            / 1000.0
        )

        min_cell_num = payload[5]

        return (
            max_cell_voltage,
            max_cell_num,
            min_cell_voltage,
            min_cell_num
        )

    def decode_0x92(self, payload):

        max_temp = payload[0] - 40
        max_temp_sensor = payload[1]

        min_temp = payload[2] - 40
        min_temp_sensor = payload[3]

        return (
            max_temp,
            max_temp_sensor,
            min_temp,
            min_temp_sensor
        )

    def decode_0x93(self, payload):

        state = payload[0]

        charge_mos = payload[1]
        discharge_mos = payload[2]

        cycles = payload[3]

        remaining_capacity_ah = (
            int.from_bytes(payload[4:8], 'big')
            / 1000.0
        )

        return (
            state,
            charge_mos,
            discharge_mos,
            cycles,
            remaining_capacity_ah
        )

    def decode_0x94(self, payload):

        num_cells = payload[0]

        num_temperature_sensors = payload[1]

        charger_connected = bool(payload[2])
        load_connected = bool(payload[3])

        io_status = payload[4]

        digital_inputs = [
            bool((io_status >> i) & 1)
            for i in range(4)
        ]

        digital_outputs = [
            bool((io_status >> (i + 4)) & 1)
            for i in range(4)
        ]

        return (
            num_cells,
            num_temperature_sensors,
            charger_connected,
            load_connected,
            digital_inputs,
            digital_outputs
        )
    
    def decode_0x95(self, payload):

        frame_num = payload[0]

        if frame_num == 0xFF:
            return None

        first_cell_index = frame_num * 3

        cells = []

        num_cells = (
            self.reported_num_cells
            if self.reported_num_cells is not None
            else self.num_cells
        )

        for i in range(3):

            cell_index = first_cell_index + i

            if cell_index >= num_cells:
                break

            start = 1 + (i * 2)

            raw_voltage = int.from_bytes(
                payload[start:start + 2],
                'big'
            )

            voltage = raw_voltage / 1000.0

            cells.append(
                (cell_index, voltage)
            )

        return frame_num, cells

    def decode_0x96(self, payload):

        frame_num = payload[0]

        if frame_num == 0xFF:
            return None

        first_sensor_index = frame_num * 7

        temperatures = []

        for i in range(7):

            sensor_index = first_sensor_index + i

            if (
                self.num_temperature_sensors is not None
                and sensor_index >= self.num_temperature_sensors
            ):
                break

            temperature = payload[i + 1] - 40

            temperatures.append(
                (sensor_index, temperature)
            )

        return frame_num, temperatures

    def decode_0x97(self, payload):

        num_cells = (
            self.reported_num_cells
            if self.reported_num_cells is not None
            else self.num_cells
        )

        balance_states = []

        for cell_index in range(num_cells):

            byte_index = cell_index // 8

            bit_index = cell_index % 8

            state = bool(
                (payload[byte_index] >> bit_index) & 1
            )

            balance_states.append(state)

        return balance_states

    def publish_battery_state(self):

        msg = BatteryState()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'base_link'

        msg.voltage = (
            float(self.voltage)
            if self.voltage is not None
            else math.nan
        )

        msg.current = (
            float(self.current)
            if self.current is not None
            else math.nan
        )

        msg.percentage = (
            self.soc / 100.0
            if self.soc is not None
            else math.nan
        )

        msg.charge = (
            self.remaining_capacity_ah
            if self.remaining_capacity_ah is not None
            else math.nan
        )

        msg.cell_voltage = [
            float(value)
            for value in self.cell_voltages
        ]

        msg.cell_temperature = [
            float(value)
            for value in self.temperatures
        ]

        msg.present = True

        self.publisher.publish(msg)

    def timer_callback(self):

        response_received = False

        for data_name in POLL_ORDER:

            match data_name:

                case 'soc':

                    payload = self.request_single_frame(
                        data_name
                    )

                    if payload is None:
                        continue

                    (
                        self.voltage,
                        self.gather_voltage,
                        self.current,
                        self.soc
                    ) = self.decode_0x90(payload)

                    response_received = True

                case 'min_max_voltage':

                    payload = self.request_single_frame(
                        data_name
                    )

                    if payload is None:
                        continue

                    (
                        self.max_cell_voltage,
                        self.max_cell_num,
                        self.min_cell_voltage,
                        self.min_cell_num
                    ) = self.decode_0x91(payload)

                    response_received = True

                case 'min_max_temp':

                    payload = self.request_single_frame(
                        data_name
                    )

                    if payload is None:
                        continue

                    (
                        self.max_temp,
                        self.max_temp_sensor,
                        self.min_temp,
                        self.min_temp_sensor
                    ) = self.decode_0x92(payload)

                    response_received = True


                case 'mos_status':

                    payload = self.request_single_frame(
                        data_name
                    )

                    if payload is None:
                        continue

                    (
                        self.bms_state,
                        self.charge_mos,
                        self.discharge_mos,
                        self.cycles,
                        self.remaining_capacity_ah
                    ) = self.decode_0x93(payload)

                    response_received = True

                case 'status_info':

                    payload = self.request_single_frame(
                        data_name
                    )

                    if payload is None:
                        continue

                    (
                        reported_cells,
                        self.num_temperature_sensors,
                        self.charger_connected,
                        self.load_connected,
                        self.digital_inputs,
                        self.digital_outputs
                    ) = self.decode_0x94(payload)

                    self.reported_num_cells = reported_cells

                    if (
                        reported_cells > 0
                        and len(self.cell_voltages)
                        != reported_cells
                    ):

                        self.cell_voltages = (
                            [math.nan] * reported_cells
                        )

                        self.balance_states = (
                            [False] * reported_cells
                        )

                    if self.num_temperature_sensors > 0:

                        if (
                            len(self.temperatures)
                            != self.num_temperature_sensors
                        ):
                            self.temperatures = (
                                [math.nan]
                                * self.num_temperature_sensors
                            )

                    response_received = True

                case 'cell_voltage':

                    num_cells = (
                        self.reported_num_cells
                        if self.reported_num_cells is not None
                        else self.num_cells
                    )

                    expected_frames = (
                        num_cells + 2
                    ) // 3

                    frames = self.request_multi_frame(
                        data_name,
                        expected_frames
                    )

                    if not frames:
                        continue

                    for payload in frames.values():

                        result = self.decode_0x95(
                            payload
                        )

                        if result is None:
                            continue

                        frame_num, cells = result

                        for cell_index, voltage in cells:

                            self.cell_voltages[
                                cell_index
                            ] = voltage

                    response_received = True

                case 'cell_temperature':

                    if (
                        self.num_temperature_sensors
                        is None
                        or self.num_temperature_sensors == 0
                    ):
                        continue

                    expected_frames = (
                        self.num_temperature_sensors + 6
                    ) // 7

                    frames = self.request_multi_frame(
                        data_name,
                        expected_frames
                    )

                    if not frames:
                        continue

                    for payload in frames.values():

                        result = self.decode_0x96(
                            payload
                        )

                        if result is None:
                            continue

                        frame_num, temperatures = result

                        for (
                            sensor_index,
                            temperature
                        ) in temperatures:

                            self.temperatures[
                                sensor_index
                            ] = temperature

                    response_received = True

                case 'cell_balance':

                    payload = self.request_single_frame(
                        data_name
                    )

                    if payload is None:
                        continue

                    self.balance_states = (
                        self.decode_0x97(payload)
                    )

                    response_received = True

        if response_received:

            self.publish_battery_state()

            self.get_logger().info(
                'BatteryState published'
            )

        else:

            self.get_logger().warning(
                'No DALY BMS responses received'
            )

    def close_socket(self):
        self.can_socket.close()


def main(args=None):

    rclpy.init(args=args)

    node = DalyBMSNode()

    try:
        rclpy.spin(node)

    finally:
        node.close_socket()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()