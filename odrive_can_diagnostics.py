#!/usr/bin/env python3
"""Passive ODrive Pro CAN diagnostic logger (classic SocketCAN; sends NO CAN messages).

Run beside ros2_control on the robot's Linux host, e.g.:
  python3 odrive_can_diagnostics.py --interface can0 --nodes 16 19 \
      --gear-ratio 140 --output odrive_plate_test.csv

Press Ctrl+C to stop. Enable relevant ODrive cyclic CAN messages separately
if you want current, temperatures, power, and error diagnostics in the CSV.
"""
import argparse
import csv
import datetime as dt
import socket
import struct
import time

FIELDS = [
    "time_iso", "time_monotonic_s", "can_id_hex", "node_id", "packet", "message_id_hex",
    "motor_position_rev", "motor_velocity_rev_s", "wheel_velocity_rad_s",
    "command_motor_velocity_rev_s", "input_torque_ff_nm",
    "iq_setpoint_a", "iq_measured_a", "fet_temperature_c", "motor_temperature_c",
    "bus_voltage_v", "bus_current_a", "torque_target_nm", "torque_estimate_nm",
    "electrical_power_w", "mechanical_power_w",
    "axis_error", "axis_state", "procedure_result", "active_errors", "disarm_reason",
]

PACKETS = {0x01: "Heartbeat", 0x03: "Get_Error", 0x09: "Get_Encoder_Estimates",
           0x0d: "Set_Input_Vel", 0x14: "Get_Iq", 0x15: "Get_Temperature",
           0x17: "Get_Bus_Voltage_Current", 0x1c: "Get_Torques", 0x1d: "Get_Powers"}


def decode(packet_id, payload, gear_ratio):
    result = {}
    if packet_id in (0x09, 0x0d, 0x14, 0x15, 0x17, 0x1c, 0x1d):
        if len(payload) < 8:
            return result
        a, b = struct.unpack_from('<ff', payload)
        keys = {
            0x09: ("motor_position_rev", "motor_velocity_rev_s"),
            0x0d: ("command_motor_velocity_rev_s", "input_torque_ff_nm"),
            0x14: ("iq_setpoint_a", "iq_measured_a"),
            0x15: ("fet_temperature_c", "motor_temperature_c"),
            0x17: ("bus_voltage_v", "bus_current_a"),
            0x1c: ("torque_target_nm", "torque_estimate_nm"),
            0x1d: ("electrical_power_w", "mechanical_power_w"),
        }
        result[keys[packet_id][0]] = a
        result[keys[packet_id][1]] = b
        if packet_id == 0x09 and gear_ratio:
            result["wheel_velocity_rad_s"] = b * 2 * 3.141592653589793 / gear_ratio
    elif packet_id == 0x01 and len(payload) >= 6:
        result["axis_error"] = struct.unpack_from('<I', payload)[0]
        result["axis_state"] = payload[4]
        result["procedure_result"] = payload[5]
    elif packet_id == 0x03 and len(payload) >= 8:
        result["active_errors"], result["disarm_reason"] = struct.unpack_from('<II', payload)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="can0")
    parser.add_argument("--nodes", type=int, nargs="+", default=[16, 19], help="ODrive CAN node IDs (decimal)")
    parser.add_argument("--gear-ratio", type=float, default=None,
                        help="Motor rev per wheel rev, e.g. 140. Only affects wheel_velocity_rad_s")
    parser.add_argument("--output", default="odrive_can_log.csv")
    args = parser.parse_args()
    if args.gear_ratio is not None and args.gear_ratio <= 0:
        parser.error("--gear-ratio must be positive")
    with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
        sock.bind((args.interface,))
        with open(args.output, "w", newline="", buffering=1) as dest:
            writer = csv.DictWriter(dest, fieldnames=FIELDS)
            writer.writeheader()
            print(f"Passively logging {args.interface} nodes {args.nodes} -> {args.output} (Ctrl+C stops)")
            try:
                while True:
                    frame = sock.recv(16)
                    if len(frame) < 16:
                        continue
                    can_id, dlc, data = struct.unpack('=IB3x8s', frame)
                    # Skip extended, remote-transmission, and error frames.
                    if can_id & 0xE0000000:
                        continue
                    can_id &= 0x7FF
                    node_id = can_id >> 5
                    packet_id = can_id & 0x1F
                    if node_id not in args.nodes or packet_id not in PACKETS:
                        continue
                    row = {"time_iso": dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds"),
                           "time_monotonic_s": f"{time.monotonic():.6f}",
                           "can_id_hex": f"0x{can_id:03X}", "node_id": node_id,
                           "message_id_hex": f"0x{packet_id:02X}", "packet": PACKETS[packet_id]}
                    row.update(decode(packet_id, data[:min(dlc, 8)], args.gear_ratio))
                    writer.writerow(row)
            except KeyboardInterrupt:
                print("\nStopped. CSV saved:", args.output)


if __name__ == '__main__':
    main()
