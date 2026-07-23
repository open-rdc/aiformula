import argparse
import bisect
from pathlib import Path

import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image
from steered_drive_msg.msg import SteeredDrive


def read_bag(bag_path: str, image_topic: str, cmd_topic: str):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (image_topic, cmd_topic):
        if topic not in type_map:
            raise RuntimeError(f'topic not found in bag: {topic}')

    images, cmds = [], []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic == image_topic:
            msg = deserialize_message(data, get_message(type_map[topic]))
            images.append((stamp_ns, msg))
        elif topic == cmd_topic:
            msg = deserialize_message(data, get_message(type_map[topic]))
            cmds.append((stamp_ns, msg))
    return images, cmds


def sync_nearest(images, cmds, max_dt_ns: int = int(0.1 * 1e9)):
    cmd_stamps = [t for t, _ in cmds]
    bridge = CvBridge()
    out_images, out_targets = [], []
    for stamp_ns, img_msg in images:
        i = bisect.bisect_left(cmd_stamps, stamp_ns)
        candidates = [c for c in (i - 1, i) if 0 <= c < len(cmd_stamps)]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda c: abs(cmd_stamps[c] - stamp_ns))
        if abs(cmd_stamps[nearest] - stamp_ns) > max_dt_ns:
            continue
        cmd_msg: SteeredDrive = cmds[nearest][1]
        bgr = bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        out_images.append(bgr)
        out_targets.append([cmd_msg.steering_angle, cmd_msg.velocity])
    return np.array(out_images, dtype=np.uint8), np.array(out_targets, dtype=np.float32)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True)
    parser.add_argument('--out', required=True)
    parser.add_argument('--image-topic', default='/zed/zed_node/rgb/image_rect_color')
    parser.add_argument('--cmd-topic', default='/cmd_vel')
    args = parser.parse_args()

    images, cmds = read_bag(args.bag, args.image_topic, args.cmd_topic)
    out_images, out_targets = sync_nearest(images, cmds)

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    np.save(out_dir / 'images.npy', out_images)
    np.save(out_dir / 'steers.npy', out_targets)
    print(f'wrote {len(out_images)} samples to {out_dir}')


if __name__ == '__main__':
    main()
