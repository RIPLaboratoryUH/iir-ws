#!/usr/bin/env python3

import socket
import struct
import time

CAN_FRAME_FORMAT = '=IB3x8s'
CAN_FRAME_SIZE   = struct.calcsize(CAN_FRAME_FORMAT)
CAN_EFF_FLAG     = 0x80000000
CAN_EFF_MASK     = 0x1FFFFFFF

addr = {
        'priority': 0x18,
        'bms': 0x01,
        'pc': 0x40
}

data_id = {
        'soc': 0x90,
        'volt_min_max': 0x91,
        'temp_min_max': 0x92,
        'MOS_status': 0x93,
        'status_info_1': 0x94,
        'cell_voltage': 0x95,
        'cell_temp': 0x96,
        'cell_balance': 0x97,
        'failure_status': 0x98
}

def initialize():

    can_socket = socket.socket(
            socket.PF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW
    )

    can_socket.bind(('can0',))
    can_socket.settimeout(0.1)

    return can_socket

def build_request(data_id):

    can_id = ( addr['priority'] << 24 ) | ( data_id[data_id] << 16 ) | ( addr['bms'] << 8 ) | ( addr['pc'] )
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

def send_request(socketcan, data_id):
    frame = build_request(data_id)
    socketcan.send(frame)

def receive_frame(socketcan):
    try:
        raw_frame = socketcan.recv(CAN_FRAME_SIZE)

        socketcan_id, data_length, payload = struct.unpack(
                CAN_FRAME_FORMAT,
                raw_frame
        )

        return socketcan_id, payload[:data_length]

    except socket.timeout:
        return None

def validate_frame(frame, data_id):
    
    can_id = frame & CAN_EFF_MASK

    expected_id = ( addr['priority'] << 24 ) | ( data_id[data_id] << 16 ) | ( addr['pc'] << 8 ) | ( addr['bms'] )

    return can_id == expected_id


def decode_0x90(payload):
    voltage = int.from_bytes(payload[0:2], 'big') / 10
    gather_voltage = int.from_bytes(payload[2:4], 'big') / 10
    current = int.from_bytes(payload[4:6], 'big') - 30000
    soc = int.from_bytes(payload[6:8], 'big') / 10 

    return voltage, gather_voltage, current, soc

def main():
    socket_can = initialize()

    try:
        while True:
            send_request(socket_can, data_id['soc'])

            deadline = time.monotonic() + 0.5
            response_received = False

            while time.monotonic() < deadline:
                result = receive_frame(socket_can)

                if result is None
                    break

                can_id, payload = result
                
                if validate_frame(can_id, data_id['soc']):
                    print(payload)
                    response_received = True
                    break
        
        if no response_received:
            print("No BMS response")

        time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping")

    finally:
        socket.close()

if __name__ == "__main__":
    main()

