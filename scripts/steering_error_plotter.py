import argparse
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


def read_caster_data(bag_path, topic_name):
    storage_options = StorageOptions(uri=bag_path, storage_id="")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topics_and_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic_name not in topics_and_types:
        raise RuntimeError(f"Topic '{topic_name}' not found in bag.")

    msg_type = get_message(topics_and_types[topic_name])

    times = []
    target = []
    measured = []
    t0 = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != topic_name:
            continue
        if t0 is None:
            t0 = t
        msg = deserialize_message(data, msg_type)
        if len(msg.data) < 2:
            continue
        times.append((t - t0) * 1e-9)
        target.append(msg.data[0])
        measured.append(msg.data[1])

    return times, target, measured


def read_linear_velocity_x(bag_path, topic_name):
    storage_options = StorageOptions(uri=bag_path, storage_id="")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topics_and_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic_name not in topics_and_types:
        raise RuntimeError(f"Topic '{topic_name}' not found in bag.")

    msg_type = get_message(topics_and_types[topic_name])

    times = []
    linear_x = []
    t0 = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic != topic_name:
            continue
        if t0 is None:
            t0 = t
        msg = deserialize_message(data, msg_type)
        times.append((t - t0) * 1e-9)
        linear_x.append(msg.twist.twist.linear.x)

    return times, linear_x


def main():
    parser = argparse.ArgumentParser(
        description="Plot /caster_data (Float64MultiArray) target/measured vs time."
    )
    parser.add_argument("bag", help="rosbag2 directory or file path")
    parser.add_argument(
        "--topic", default="/caster_data", help="topic name (default: /caster_data)"
    )
    parser.add_argument(
        "--out",
        default=None,
        help="output PNG path (default: <bag>_caster_data.png)",
    )
    parser.add_argument(
        "--velocity-topic",
        default="/vectornav/velocity_body",
        help="TwistWithCovarianceStamped topic name (default: /vectornav/velocity_body)",
    )
    parser.add_argument(
        "--velocity-out",
        default=None,
        help=(
            "linear.x output PNG path (default: <bag>_velocity_body_linear_x.png)"
        ),
    )
    args = parser.parse_args()

    bag_path = args.bag
    out_path = args.out
    if out_path is None:
        base = os.path.basename(os.path.normpath(bag_path))
        out_path = os.path.join(os.path.dirname(bag_path), f"{base}_caster_data.png")
    velocity_out_path = args.velocity_out
    if velocity_out_path is None:
        base = os.path.basename(os.path.normpath(bag_path))
        velocity_out_path = os.path.join(
            os.path.dirname(bag_path), f"{base}_velocity_body_linear_x.png"
        )

    try:
        times, target, measured = read_caster_data(bag_path, args.topic)
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

    if not times:
        print("No valid messages found to plot.", file=sys.stderr)
        return 1

    target_deg = [math.degrees(value) for value in target]
    measured_deg = [math.degrees(value) for value in measured]

    plt.figure(figsize=(10, 5))
    plt.plot(times, target_deg, label="Target δ")
    plt.plot(times, measured_deg, label="Measured δ")
    plt.xlabel("Time [s]")
    plt.ylabel("Steering Angle δ [deg]")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()

    try:
        velocity_times, velocity_linear_x = read_linear_velocity_x(
            bag_path, args.velocity_topic
        )
    except Exception as e:
        print(f"Velocity plot skipped: {e}", file=sys.stderr)
    else:
        if velocity_times:
            plt.figure(figsize=(10, 5))
            plt.plot(velocity_times, velocity_linear_x, label="Linear velocity x")
            plt.xlabel("Time [s]")
            plt.ylabel("Linear Velocity X [m/s]")
            plt.grid(True, linestyle="--", alpha=0.5)
            plt.legend()
            plt.tight_layout()
            plt.savefig(velocity_out_path, dpi=150)
            plt.close()
        else:
            print("Velocity plot skipped: no valid messages found.", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
