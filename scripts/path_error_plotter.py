import rosbag2_py
from sensor_msgs.msg import NavSatFix
from rclpy.serialization import deserialize_message
import math
import csv
import matplotlib.pyplot as plt

class GnssDataProcessor:
    def __init__(self, bag_file_path, target_csv_path):
        self.target_path = self.load_target_path(target_csv_path)  # CSVファイル名を指定

        self.accumulated_error = 0.0

        self.tracked_path, self.total_duration, self.average_frequency = self.read_rosbag_data(bag_file_path)

        self.fig, self.ax = plt.subplots()

        self.plot_data()    # プロット

    def load_target_path(self, file_path):
        path = []
        with open(file_path, 'r') as file:
            reader = csv.reader(file)
            next(reader)  # ヘッダー行をスキップ
            for row in reader:
                lat = float(row[0])
                lon = float(row[1])
                path.append((lat, lon))
        return path

    def read_rosbag_data(self, bag_file_path):
        tracked_path = []
        first_timestamp = None
        last_timestamp = None
        intervals = []

        # SequentialReaderを作成
        reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )
        reader.open(storage_options, converter_options)

        # トピック情報を取得
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}

        # /gps/fixトピックをフィルタリング
        topic_name = '/vectornav/gnss'  # GNSSデータのトピック名に合わせて変更
        if topic_name not in type_map:
            print(f"Topic '{topic_name}' not found in the bag file.")
            return tracked_path, 0.0, 0.0

        previous_timestamp = None

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            if topic == topic_name:
                # 最初と最後のタイムスタンプを記録
                if first_timestamp is None:
                    first_timestamp = t
                last_timestamp = t

                # NavSatFixメッセージのデシリアライズ
                msg = deserialize_message(data, NavSatFix)
                tracked_path.append((msg.latitude, msg.longitude))

                # メッセージ間隔を計算
                if previous_timestamp is not None:
                    interval = (t - previous_timestamp) * 1e-9  # 秒単位
                    intervals.append(interval)

                previous_timestamp = t

        # 累積時間を計算 (秒単位)
        total_duration = (last_timestamp - first_timestamp) * 1e-9 if first_timestamp and last_timestamp else 0.0

        # 平均周波数を計算 (Hz)
        if intervals:
            average_interval = sum(intervals) / len(intervals)
            average_frequency = 1.0 / average_interval if average_interval > 0 else 0.0
        else:
            average_frequency = 0.0

        return tracked_path, total_duration, average_frequency

    def calculate_min_distance_to_path(self, lat, lon):
        min_distance = float('inf')

        # 経路の各セグメントを調べ、最短距離を見つける
        for i in range(len(self.target_path) - 1):
            start = self.target_path[i]
            end = self.target_path[i + 1]

            # 現在の位置からセグメントへの最短距離を計算
            distance = self.calculate_distance_to_segment(lat, lon, start, end)
            min_distance = min(min_distance, distance)

        return min_distance

    def calculate_distance_to_segment(self, lat, lon, start, end):
        # セグメントの端点の座標
        lat1, lon1 = start
        lat2, lon2 = end

        # 現在の位置とセグメントの端点の距離を計算
        dist_start_to_point = self.calculate_distance(lat1, lon1, lat, lon)
        dist_end_to_point = self.calculate_distance(lat2, lon2, lat, lon)
        dist_start_to_end = self.calculate_distance(lat1, lon1, lat2, lon2)

        # 内積を使用して点がセグメント上にあるかを判定
        if dist_start_to_end == 0:
            # セグメントの始点と終点が同じ場合
            return dist_start_to_point

        # 点がセグメントの外側にある場合
        t = ((lat - lat1) * (lat2 - lat1) + (lon - lon1) * (lon2 - lon1)) / (dist_start_to_end ** 2)
        if t < 0.0:
            return dist_start_to_point
        elif t > 1.0:
            return dist_end_to_point

        # 点がセグメントの内側にある場合、垂線の距離を計算
        proj_lat = lat1 + t * (lat2 - lat1)
        proj_lon = lon1 + t * (lon2 - lon1)
        return self.calculate_distance(lat, lon, proj_lat, proj_lon)

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371000  # 地球の半径 (メートル)

        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)

        a = math.sin(delta_lat / 2) ** 2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        distance = R * c
        return distance

    def plot_data(self):
        target_lats = [lat for lat, lon in self.target_path]
        target_lons = [lon for lat, lon in self.target_path]
        tracked_lats = [lat for lat, lon in self.tracked_path]
        tracked_lons = [lon for lat, lon in self.tracked_path]

        # GNSSデータからの誤差を計算
        for lat, lon in self.tracked_path:
            min_distance = self.calculate_min_distance_to_path(lat, lon)
            self.accumulated_error += min_distance

        # プロット設定
        self.ax.plot(target_lons, target_lats, 'bo-', label='Target Path')
        self.ax.plot(tracked_lons, tracked_lats, 'ro-', label='Tracked Path')
        self.ax.set_title('GNSS Tracking & Target Path')
        self.ax.set_xlabel('Longitude')
        self.ax.set_ylabel('Latitude')
        self.ax.legend(loc='upper right')

        # 累積誤差、累積時間、平均周波数を表示
        self.ax.text(0.02, 0.95, f'Accumulated error: {self.accumulated_error:.2f} m', transform=self.ax.transAxes)
        self.ax.text(0.02, 0.90, f'Total duration: {self.total_duration:.2f} sec', transform=self.ax.transAxes)
        self.ax.text(0.02, 0.85, f'Average frequency: {self.average_frequency:.2f} Hz', transform=self.ax.transAxes)

        print(f'1データごとの平均誤差: {self.accumulated_error/(self.total_duration*self.average_frequency):.2f} m')

        plt.show()

def main():
    # rosbagのパスと目標CSVファイルのパスを指定
    bag_file_path = '/home/kazuma/rosbag/240910_shihou_rosbag/rosbag2_2024_09_10-15_39_39'  # ここにrosbagファイルのパスを入力
    target_csv_path = 'gnssnav/config/course_data/shihou_full.csv'  # ここに目標経路のCSVファイルのパスを入力

    GnssDataProcessor(bag_file_path, target_csv_path)

if __name__ == '__main__':
    main()