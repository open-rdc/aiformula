# pose_estimater

## Package overview

ベクトルマップと車線境界線点群を用いたICP（Iterative Closest Point）による自己位置推定ノードです。GNSS/IMUから得た概算姿勢（map座標系）を初期値として、[`lane_line_publisher`](../../perception/lane_line_publisher)が配信する車線境界線点群を[`vectormap_server`](../../map/vectormap_server)から配信されるベクトルマップに位置合わせすることで、より高精度な自己位置を求めます。推定結果は[`ekf_localizer`](../ekf_localizer)の`/localization/icp_pose`入力として利用されます。

## Nodes

### pose_estimater_node

#### Input

| **Name（Topic）**             | **Type**                                | **Description**                                                             |
| -------------------------------- | ------------------------------------------ | ----------------------------------------------------------------------------- |
| `/perception/lane_line_points`     | `sensor_msgs/msg/PointCloud2`             | 観測された車線境界線点群（ICPの入力点群）                                       |
| `/vector_map`                      | `vectormap_msgs/msg/VectorMap`            | ベクトルマップ。線分を`map_sample_interval_m`間隔でリサンプルしICPターゲットを構築 |
| `/vectornav/gnss`                   | `sensor_msgs/msg/NavSatFix`               | GNSS生データ。map座標系での概算位置算出に使用                                    |
| `/vectornav/imu`                    | `sensor_msgs/msg/Imu`                     | IMU姿勢。概算yaw算出に使用                                                     |

#### Output

| **Name（Topic）**            | **Type**                                       | **Description**                                          |
| -------------------------------- | -------------------------------------------------- | ------------------------------------------------------------ |
| `/localization/icp_pose`           | `geometry_msgs/msg/PoseWithCovarianceStamped`      | ICPで補正された自己位置（収束しない場合はGNSS/IMUのみによるフォールバック値） |
| `/localization/pose_raw`           | `geometry_msgs/msg/PoseWithCovarianceStamped`      | ICP補正前のGNSS/IMUベースの概算自己位置                        |

#### Parameters

| **Name（Parameter）**                     | **Type** | **Description**                                                         |
| ------------------------------------------- | -------- | --------------------------------------------------------------------------- |
| `interval_ms`                                 | `int`    | メイン処理タイマーの周期[ms]                                                 |
| `input_timeout_s`                             | `double` | GNSS/IMU入力が有効とみなされる最大経過時間[s]                                 |
| `map_origin_geodetic.latitude`                | `double` | map座標系原点の緯度                                                          |
| `map_origin_geodetic.longitude`               | `double` | map座標系原点の経度                                                          |
| `map_yaw_from_east`                            | `double` | map座標系のX軸が真東からなす角度[rad]                                          |
| `min_observed_points`                          | `int`    | ICPを試行するために必要な観測点群の最小点数                                     |
| `map_sample_interval_m`                        | `double` | ベクトルマップの線分をICPターゲット点群にリサンプルする間隔[m]                    |
| `gnss_position_variance`                       | `double` | GNSSベースの位置（raw pose、およびICP未収束時のフォールバック）の分散            |
| `imu_yaw_variance`                             | `double` | IMUベースのyawの分散                                                         |
| `icp_position_variance`                        | `double` | ICP補正後の位置共分散のベーススケール                                          |
| `icp.max_iterations`                           | `int`    | ICPの最大反復回数                                                            |
| `icp.max_correspondence_distance`              | `double` | ICPで対応点とみなす最大距離[m]                                                |
| `icp.convergence_translation_epsilon`          | `double` | ICP収束判定の並進閾値                                                        |
| `icp.min_correspondences`                      | `int`    | ICP結果を有効とみなすために必要な最小対応点数                                   |
| `icp.max_mean_error`                           | `double` | ICP収束を受理する平均誤差の上限                                                |
