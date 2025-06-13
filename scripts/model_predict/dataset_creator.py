#!/usr/bin/env python3

import csv
import rclpy
import struct
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from socketcan_interface_msg.msg import SocketcanIF
from collections import deque

def bytes_to_short(byte_list):
    return struct.unpack('<h', bytearray(byte_list[:2]))[0]

def read_bag_and_export_csv(bag_path, output_csv):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    reader.open(storage_options, converter_options)

    valid_topics = {
        '/vectornav/imu': Imu,
        '/vectornav/velocity_body': TwistWithCovarianceStamped,
        '/can_rx_11': SocketcanIF,
        '/cmd_vel': Twist
    }

    type_map = {}
    for topic in reader.get_all_topics_and_types():
        if topic.name in valid_topics:
            type_map[topic.name] = get_message(topic.type)

    latest_vel = None
    latest_acc = None
    latest_potentio = None
    latest_cmd = None

    # 保存用
    history = deque(maxlen=2)
    step = 0

    with open(output_csv, mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow([
            'step',
            'v0', 'w0', 'acc_v0', 'acc_w0', 'potentio_th0',
            'linear_x', 'angular_z',
            'v1', 'w1', 'potentio_th1'
        ])

        while reader.has_next():
            topic, data, t = reader.read_next()
            if topic not in type_map:
                continue
            msg_type = type_map[topic]
            msg = deserialize_message(data, msg_type)

            if topic == '/vectornav/imu':
                latest_acc = {
                    'acc_v': msg.linear_acceleration.x,
                    'acc_w': msg.angular_velocity.z
                }

            elif topic == '/vectornav/velocity_body':
                latest_vel = {
                    'v': msg.twist.twist.linear.x,
                    'w': msg.twist.twist.angular.z
                }

            elif topic == '/can_rx_11':
                latest_potentio = {
                    'potentio_th': bytes_to_short(msg.candata[:2])
                }

            elif topic == '/cmd_vel':
                latest_cmd = {
                    'linear_x': msg.linear.x,
                    'angular_z': msg.angular.z
                }

                # すべてのセンサが揃っているときのみ記録
                if latest_vel and latest_acc and latest_potentio and latest_cmd:
                    record = {
                        'v': latest_vel['v'],
                        'w': latest_vel['w'],
                        'acc_v': latest_acc['acc_v'],
                        'acc_w': latest_acc['acc_w'],
                        'potentio_th': latest_potentio['potentio_th'],
                        'linear_x': latest_cmd['linear_x'],
                        'angular_z': latest_cmd['angular_z']
                    }
                    history.append(record)

                    if len(history) == 2:
                        x = history[0]
                        y = history[1]
                        row = [
                            step,
                            x['v'], x['w'], x['acc_v'], x['acc_w'], x['potentio_th'],
                            x['linear_x'], x['angular_z'],
                            y['v'], y['w'], y['potentio_th']
                        ]
                        writer.writerow(row)
                        step += 1

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', type=str, help='Path to ROS2 bag')
    parser.add_argument('--output', type=str, default='output_lstm.csv', help='CSV output path')
    args = parser.parse_args()

    rclpy.init()
    read_bag_and_export_csv(args.bag_path, args.output)
    rclpy.shutdown()
