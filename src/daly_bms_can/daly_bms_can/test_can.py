#!/usr/bin/env python3

import socket
import struct

PRIORITY = 0x18
BMS_ADDR = 0x01
PC_ADDR  = 0x40

CAN_FRAME_FORMAT = '=IB3x8s'
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FORMAT)
CAN_EFF_FLAG = 0X80000000

def initialize():

    can_socket = socket.socket(
            socket.PF_CAN,
            socket.SOCK_RAW,
            socket.CAN_RAW
    )

    can_socket.bind(('can0',))
    can_socket.settimeout(0.1)

    return can_socket

def build_request():
    data_id = DALY_IDS[data_name]

    can_id = (PRIORITY << 24) | (0x90 << 16) | (BMS_ADDR << 8) | (PC_ADDR)
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

def send_request(socketcan):
    frame = build_request()
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

def validate_frame(frame):

    can_id = frame & CAN_EFF_MASK

    expected_id = (PRIORITY << 24) | (0x90 << 16) | (PC_ADDR << 8) | (BMS_ADDR)

    if can_id != expected_id:
        return None

def main():

    socket = initialize()

    send_request(socket)

    print(receive_frame)

if __name__ == "__main__":
    main()
