# simulator

## Package overview

Gazebo（Ignition/GZ Sim）によるシミュレーション環境のbringupパッケージです。コース世界（`shihou_world.sdf`）と車両モデル（`ai_car1`、差動駆動＋`ros2_control`によるキャスター操舵、IMU/カメラ/深度カメラ/GNSSを搭載）を起動し、`ros_gz_bridge`でGazebo側のトピックをROS2側へ橋渡しします。あわせて、実車の[`vectornav`](../sensing/vectornav)・ZEDカメラと同じトピック形状になるよう変換する3つのPythonノードを起動し、実機用のスタックをシミュレーション上でそのまま動作させられるようにしています。

## Launch

`launch/gazebo_ignition.launch.py`

| **Launch argument** | **Default**          | **Description**            |
| ---------------------- | ----------------------- | -------------------------------- |
| `world`                  | `shihou_world.sdf`       | `world/`ディレクトリ以下のワールドSDFファイル名 |

## `ros_gz_bridge`によるトピック橋渡し

| **Gazebo topic**              | **ROS2 topic（リマップ後）**                  | **Type**                             | **Description**                  |
| -------------------------------- | ---------------------------------------------- | ---------------------------------------- | --------------------------------------- |
| `/clock`                          | `/clock`                                        | `rosgraph_msgs/msg/Clock`                 | シミュレーション時刻                        |
| `/camera_info`                    | `/camera_info`                                  | `sensor_msgs/msg/CameraInfo`              | RGBカメラの内部パラメータ                    |
| `/image_raw`                      | `/zed/zed_node/rgb/image_rect_color`             | `sensor_msgs/msg/Image`                   | RGBカメラ画像（実機ZEDと同じトピック名に合わせている） |
| `/depth_image_raw/depth_image`    | `/zed/zed_node/depth/depth_registered`（想定）    | `sensor_msgs/msg/Image`                   | 深度画像                                   |
| `/depth_image_raw/points`         | `/zed/zed_node/pointcloud`                      | `sensor_msgs/msg/PointCloud2`             | 深度点群                                   |
| `/odom`                           | `/odom`                                         | `nav_msgs/msg/Odometry`                   | `DiffDrive`プラグインによるオドメトリ         |
| `/navsat`                         | `/navsat`                                       | `sensor_msgs/msg/NavSatFix`               | GPSセンサ出力                              |
| `/imu_raw`                        | `/imu_raw`                                      | `sensor_msgs/msg/Imu`                     | IMUセンサ出力                              |
| `/cmd_vel_twist`                  | （ROS→Gazebo）                                  | `geometry_msgs/msg/Twist`                 | `DiffDrive`プラグインへの速度指令              |

> **Note:** `remappings`に定義された`/depth_image`というトピック名が、実際に橋渡しされる`/depth_image_raw/depth_image`と一致していないように見えます。実行時は`ros2 topic list`で実際のトピック名を確認してください。

## Nodes（`scripts/`）

モビリティのVectorNav・走行コマンドと同じトピック形状に変換するヘルパーノードです。

### steered_to_twist（`scripts/steered_to_twist.py`）

`cmd_vel`（`SteeredDrive`）をGazeboの`DiffDrive`プラグイン用の`Twist`に変換します。

| **Name（Topic、パラメータで変更可）** | **Type**                                | **方向** | **Description**            |
| ---------------------------------------- | ------------------------------------------- | ---------- | -------------------------------- |
| `/cmd_vel`（既定値）                        | `steered_drive_msg/msg/SteeredDrive`         | Input      | 操舵角＋速度の走行コマンド              |
| `/cmd_vel_twist`（既定値）                   | `geometry_msgs/msg/Twist`                    | Output     | `DiffDrive`プラグイン用の速度指令       |
| `/cmd_caster`（既定値）                      | `std_msgs/msg/Float64MultiArray`             | Output     | キャスター操舵角指令（`caster_yaw_position_controller`用） |

| **Name（Parameter）** | **Type / Default**  | **Description**              |
| ------------------------- | ---------------------- | ---------------------------------- |
| `wheel_base`                 | `double` / `0.8`         | 自転車モデル変換に用いるホイールベース[m] |
| `input_topic`                 | `string` / `/cmd_vel`    | 入力トピック名                        |
| `output_topic`                | `string` / `/cmd_vel_twist` | 出力トピック名（Twist）                |
| `caster_topic`                | `string` / `/cmd_caster` | 出力トピック名（キャスター指令）          |

### convert_sim_to_vectornav_pose（`scripts/convert_sim_to_vectornav_pose.py`）

シミュレータのIMU・GNSSを実機VectorNavと同じトピック形状に変換します。

| **Name（Topic）**  | **Type**                                    | **方向** | **Description**            |
| ---------------------- | ---------------------------------------------- | ---------- | -------------------------------- |
| `/imu_raw`               | `sensor_msgs/msg/Imu`                          | Input      | シミュレータのIMU出力                   |
| `/navsat`                | `sensor_msgs/msg/NavSatFix`                    | Input      | シミュレータのGNSS出力                   |
| `/vectornav/imu`          | `sensor_msgs/msg/Imu`                          | Output     | yawオフセット補正済みIMU                 |
| `/vectornav/gnss`         | `sensor_msgs/msg/NavSatFix`                    | Output     | GNSS（パススルー）                       |
| `/vectornav/pose`         | `geometry_msgs/msg/PoseWithCovarianceStamped`  | Output     | ECEF座標系の位置＋補正済み姿勢               |

| **Name（Parameter）** | **Type / Default**    | **Description**                     |
| ------------------------- | ------------------------ | ------------------------------------------ |
| `yaw_offset_deg`             | `double` / `-176.0`        | IMU yawに加えるオフセット角[deg]              |
| `imu_frame_id`                | `string` / `base_link`     | 出力する各メッセージの`frame_id`（launchでは`vectornav`を指定） |

### convert_sim_to_vectornav_velocity_body（`scripts/convert_sim_to_vectornav_velocity_body.py`）

シミュレータのオドメトリを実機VectorNavの速度トピック形状に変換します。

| **Name（Topic）**          | **Type**                                       | **方向** | **Description**       |
| ------------------------------ | --------------------------------------------------- | ---------- | ---------------------------- |
| `/odom`                          | `nav_msgs/msg/Odometry`                             | Input      | シミュレータのオドメトリ           |
| `/vectornav/velocity_body`        | `geometry_msgs/msg/TwistWithCovarianceStamped`      | Output     | 車体座標系での速度                |

| **Name（Parameter）** | **Type / Default**  | **Description**                                |
| ------------------------- | ---------------------- | ------------------------------------------------------ |
| `frame_id`                   | `string` / `vectornav`   | 出力メッセージの`frame_id`                                  |
