import os
import sys
import rosbag2_py
from sensor_msgs.msg import NavSatFix
from rclpy.serialization import deserialize_message
from ament_index_python.packages import get_package_share_directory
import math
import csv
import matplotlib.pyplot as plt
from pyproj import Transformer

class GnssDataProcessor:
    def __init__(self, bag_file_path, target_csv_path):
        self.target_path = self.load_target_path(target_csv_path)
        self.tracked_path, self.total_duration, self.average_frequency = self.read_rosbag_data(bag_file_path)

        self.transformer = None
        if self.tracked_path:
            lat, lon = self.tracked_path[0]
            self.set_utm_transformer(lat, lon)

        self.errors = []

        # 経路グラフを描画
        self.plot_path()
        self.plot_error()

    def set_utm_transformer(self, lat, lon):
        utm_zone = int((lon + 180) / 6) + 1
        self.transformer = Transformer.from_crs("EPSG:4326", f"EPSG:326{utm_zone}", always_xy=True)

    def load_target_path(self, file_path):
        path = []
        try:
            with open(file_path, 'r') as file:
                reader = csv.reader(file)
                next(reader)
                for row in reader:
                    lat = float(row[0])
                    lon = float(row[1])
                    path.append((lat, lon))
        except FileNotFoundError:
            print(f"指定された目標経路のファイルが見つかりません: {file_path}")
        return path

    def read_rosbag_data(self, bag_file_path):
        tracked_path = []
        first_timestamp = None
        last_timestamp = None
        intervals = []

        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}

        topic_name = '/vectornav/gnss'  # gnssトピックの指定
        if topic_name not in type_map:
            print(f"Topic '{topic_name}' not found in the bag file.")
            return tracked_path, 0.0, 0.0

        previous_timestamp = None

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == topic_name:
                if first_timestamp is None:
                    first_timestamp = t
                last_timestamp = t

                msg = deserialize_message(data, NavSatFix)
                tracked_path.append((msg.latitude, msg.longitude))

                if previous_timestamp is not None:
                    interval = (t - previous_timestamp) * 1e-9
                    intervals.append(interval)

                previous_timestamp = t

        total_duration = (last_timestamp - first_timestamp) * 1e-9 if first_timestamp and last_timestamp else 0.0
        average_frequency = 1.0 / (sum(intervals) / len(intervals)) if intervals else 0.0

        return tracked_path, total_duration, average_frequency

    def calculate_min_distance_to_path(self, lat, lon):
        min_distance = float('inf')

        for i in range(len(self.target_path) - 1):
            start = self.target_path[i]
            end = self.target_path[i + 1]
            distance = self.calculate_distance_to_segment(lat, lon, start, end)
            min_distance = min(min_distance, distance)

        return min_distance

    def calculate_distance_to_segment(self, lat, lon, start, end):
        x1, y1 = self.transformer.transform(start[1], start[0])
        x2, y2 = self.transformer.transform(end[1], end[0])
        x0, y0 = self.transformer.transform(lon, lat)

        dx, dy = x2 - x1, y2 - y1
        t = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx ** 2 + dy ** 2)

        if t < 0:
            closest_x, closest_y = x1, y1
        elif t > 1:
            closest_x, closest_y = x2, y2
        else:
            closest_x, closest_y = x1 + t * dx, y1 + t * dy

        return math.sqrt((x0 - closest_x) ** 2 + (y0 - closest_y) ** 2)

    def plot_path(self):
        target_lats = [lat for lat, lon in self.target_path]
        target_lons = [lon for lat, lon in self.target_path]
        tracked_lats = [lat for lat, lon in self.tracked_path]
        tracked_lons = [lon for lat, lon in self.tracked_path]

        accumulated_error = 0
        max_error = 0
        max_error_coord = ()
        datasize = 0

        for i, (lat, lon) in enumerate(self.tracked_path):
            min_distance = self.calculate_min_distance_to_path(lat, lon)
            accumulated_error += min_distance
            if min_distance > max_error:
                max_error_coord = (lon, lat)
                max_error = min_distance
            datasize += 1
            self.errors.append(min_distance)

        plt.figure()
        plt.plot(target_lons, target_lats, 'bo-', label='Target Path')
        plt.plot(tracked_lons, tracked_lats, 'ro-', label='Tracked Path')
        plt.plot(max_error_coord[0], max_error_coord[1], 'gx', ms=10, label='Max Error')
        plt.plot(tracked_lons[0], tracked_lats[0], 'ko', ms=5, label='Start')
        plt.plot(tracked_lons[-1], tracked_lats[-1], 'kx', ms=5, label='Goal')

        plt.title('GNSS Tracking & Target Path')
        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.legend(loc='upper right')

        # axで相対位置に表示させる
        ax = plt.gca()
        ax.text(0.02, 0.95, f'Accumulated error: {accumulated_error:.2f} m', transform=ax.transAxes, fontsize=10)
        ax.text(0.02, 0.90, f'Total duration: {self.total_duration:.2f} sec', transform=ax.transAxes, fontsize=10)
        ax.text(0.02, 0.85, f'Average frequency: {self.average_frequency:.2f} Hz', transform=ax.transAxes, fontsize=10)
        ax.text(0.02, 0.80, f'Max error: {max_error:.2f} m', transform=ax.transAxes, fontsize=10)

    def plot_error(self):
        plt.figure()
        errors = self.errors

        plt.plot(errors, 'r-', label='Error Distance (m)')
        plt.title('Errors')
        plt.xlabel('Data Point Index')
        plt.ylabel('Error Distance (m)')
        plt.legend(loc='upper right')

        print(f'1データごとの平均誤差: {sum(errors) / len(errors):.2f} m')

        plt.show()

def main():
    if len(sys.argv) < 2:
        print("rosbagのファイルパスを引数として渡してください")
        sys.exit(1)

    bag_file_path = sys.argv[1] # rosbag
    target_csv_path = os.path.join(
        get_package_share_directory('gnssnav'),
        'config',
        'course_data',
        'shihou_241030_dynamic.csv'
    )   # 目標csv

    GnssDataProcessor(bag_file_path, target_csv_path)

if __name__ == '__main__':
    main()
