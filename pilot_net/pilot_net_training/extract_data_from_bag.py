import argparse
from pathlib import Path

import numpy as np
import rosbag2_py
from cv_bridge import CvBridge
from numpy.lib.format import open_memmap
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

IMAGE_TOPIC = '/zed/zed_node/rgb/image_rect_color'
CMD_TOPIC = '/cmd_vel'
MAX_DT_NS = int(0.1 * 1e9)


def open_reader(bag_path: str, topics: list[str]):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, rosbag2_py.ConverterOptions('', ''))

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in topics:
        if topic not in type_map:
            raise RuntimeError(f'topic not found in bag: {bag_path}: {topic}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader, type_map


def scan_bag(bag_path: str, image_topic: str, cmd_topic: str):
    """画像は展開せずタイムスタンプだけ集め、cmd は舵角まで読む。

    画像本体をメモリに載せずに同期対応を決めるための1パス目。
    """
    reader, type_map = open_reader(bag_path, [image_topic, cmd_topic])
    cmd_type = get_message(type_map[cmd_topic])

    image_stamps, cmd_stamps, steers = [], [], []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic == image_topic:
            image_stamps.append(stamp_ns)
        else:
            cmd_stamps.append(stamp_ns)
            steers.append(deserialize_message(data, cmd_type).steering_angle)

    return (np.array(image_stamps, dtype=np.int64),
            np.array(cmd_stamps, dtype=np.int64),
            np.array(steers, dtype=np.float32))


def match_nearest(image_stamps: np.ndarray, cmd_stamps: np.ndarray,
                  max_dt_ns: int = MAX_DT_NS):
    """各画像に最も時刻の近い cmd を割り当て、(画像index, cmd index) を返す。

    max_dt_ns を超えるものは捨てる。等距離のときは古い方の cmd を採る。
    """
    if len(image_stamps) == 0 or len(cmd_stamps) == 0:
        empty = np.empty(0, dtype=np.int64)
        return empty, empty.copy()

    right = np.clip(np.searchsorted(cmd_stamps, image_stamps), 0, len(cmd_stamps) - 1)
    left = np.clip(right - 1, 0, len(cmd_stamps) - 1)
    dt_left = np.abs(cmd_stamps[left] - image_stamps)
    dt_right = np.abs(cmd_stamps[right] - image_stamps)

    nearest = np.where(dt_left <= dt_right, left, right)
    valid = np.minimum(dt_left, dt_right) <= max_dt_ns
    return np.flatnonzero(valid).astype(np.int64), nearest[valid].astype(np.int64)


def peek_image_shape(bag_path: str, image_topic: str) -> tuple[int, int, int]:
    reader, type_map = open_reader(bag_path, [image_topic])
    while reader.has_next():
        _, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(type_map[image_topic]))
        return CvBridge().imgmsg_to_cv2(msg, desired_encoding='bgr8').shape
    raise RuntimeError(f'no image message in bag: {bag_path}')


def write_images(bag_path: str, image_topic: str, image_idx: np.ndarray,
                 out_images: np.ndarray, offset: int):
    """2パス目。採用した画像だけを展開して出力配列へ直接書き込む。"""
    reader, type_map = open_reader(bag_path, [image_topic])
    image_type = get_message(type_map[image_topic])
    bridge = CvBridge()

    position, taken = 0, 0
    while reader.has_next() and taken < len(image_idx):
        _, data, _ = reader.read_next()
        if position == image_idx[taken]:
            msg = deserialize_message(data, image_type)
            out_images[offset + taken] = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            taken += 1
        position += 1
    if taken != len(image_idx):
        raise RuntimeError(f'expected {len(image_idx)} images from {bag_path}, wrote {taken}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', required=True, nargs='+')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()

    matches = []
    for bag in args.bag:
        image_stamps, cmd_stamps, steers = scan_bag(bag, IMAGE_TOPIC, CMD_TOPIC)
        image_idx, cmd_idx = match_nearest(image_stamps, cmd_stamps)
        matches.append((bag, image_idx, steers[cmd_idx]))
        print(f'{bag}: {len(image_idx)}/{len(image_stamps)} images matched')

    total = sum(len(image_idx) for _, image_idx, _ in matches)
    if total == 0:
        raise RuntimeError('no synchronised samples found in the given bags')

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_images = open_memmap(out_dir / 'images.npy', mode='w+', dtype=np.uint8,
                             shape=(total, *peek_image_shape(args.bag[0], IMAGE_TOPIC)))
    out_steers = open_memmap(out_dir / 'steers.npy', mode='w+', dtype=np.float32,
                             shape=(total, 1))

    offset = 0
    for bag, image_idx, steers in matches:
        out_steers[offset:offset + len(image_idx), 0] = steers
        write_images(bag, IMAGE_TOPIC, image_idx, out_images, offset)
        offset += len(image_idx)

    out_images.flush()
    out_steers.flush()
    print(f'wrote {total} samples to {out_dir}')


if __name__ == '__main__':
    main()
