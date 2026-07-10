# ekf_localizer

## Package overview

Extended Kalman Filter（EKF）による自己位置推定と、それに付随するTF配信を行うパッケージです。以下の3つのノードで構成されます。

* `ekf_localizer_node`: [`pose_estimater`](../pose_estimater)から得られるICPベースの自己位置観測と、VectorNavの速度・角速度を用いてEKFを実行し、map座標系での自己位置（`/localization/pose`）を推定します。
* `odom_tf_node`: IMUのyawとVectorNavの車体速度からオドメトリを積分し、`odom` -> `base_link` のTFおよび`/localization/odom`を配信します。
* `map_odom_tf_node`: EKFで推定したmap座標系での自己位置と`odom` -> `base_link` のTFから、`map` -> `odom` のTFを算出し配信します。

## Nodes

### ekf_localizer_node

#### Input

| **Name（Topic）**        | **Type**                                          | **Description**                                                        |
| ---------------------- | -------------------------------------------------- | ------------------------------------------------------------------------ |
| `/localization/icp_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped`       | `pose_estimater`から得られるICPベースの自己位置観測（更新ステップに使用） |
| `/vectornav/velocity_body` | `geometry_msgs/msg/TwistWithCovarianceStamped`    | VectorNavから得られる車体座標系での速度・角速度（予測ステップに使用）    |

#### Output

| **Name（Topic）**   | **Type**                                    | **Description**                          |
| ------------------ | --------------------------------------------- | ------------------------------------------- |
| `/localization/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | EKFにより推定されたmap座標系での自己位置 |

#### Parameters

| **Name（Parameter）**       | **Type** | **Description**                                                              |
| -------------------------- | -------- | -------------------------------------------------------------------------------- |
| `input_timeout_s`           | `double` | `/localization/icp_pose`の入力が途絶したとみなすタイムアウト時間[s]。超過するとpose publishを停止する |
| `predict_interval_ms`       | `int`    | EKFの予測ステップを実行する周期[ms]                                              |
| `tf_interval_ms`            | `int`    | `/localization/pose` をpublishする周期[ms]                                       |
| `initial_position_variance` | `double` | 初期化時の位置の分散                                                            |
| `initial_yaw_variance`      | `double` | 初期化時のyawの分散                                                             |
| `process_position_variance` | `double` | 予測ステップにおける位置のプロセスノイズ分散                                     |
| `process_yaw_variance`      | `double` | 予測ステップにおけるyawのプロセスノイズ分散                                      |
| `process_velocity_variance` | `double` | 予測ステップにおける速度のプロセスノイズ分散                                     |
| `process_yaw_rate_variance` | `double` | 予測ステップにおける角速度のプロセスノイズ分散                                   |
| `position_gate_dist`        | `double` | 位置観測更新を採択するマハラノビス距離ゲートの閾値                               |
| `yaw_gate_dist`              | `double` | yaw観測更新を採択するマハラノビス距離ゲートの閾値                                |

### odom_tf_node

#### Input

| **Name（Topic）**          | **Type**                                       | **Description**                                                                    |
| -------------------------- | ------------------------------------------------- | -------------------------------------------------------------------------------------- |
| `/vectornav/imu`             | `sensor_msgs/msg/Imu`                             | yaw角の取得に用いるIMUデータ                                                          |
| `/vectornav/velocity_body`   | `geometry_msgs/msg/TwistWithCovarianceStamped`    | 車体座標系での速度・角速度。`frame_id`は`base_link`または`vectornav`である必要がある |

#### Output

| **Name（Topic / TF）**     | **Type**                          | **Description**                                    |
| --------------------------- | ------------------------------------ | ------------------------------------------------------ |
| `/localization/odom`         | `nav_msgs/msg/Odometry`             | 速度積分によって求めたodom座標系での自己位置・速度 |
| TF: `odom` -> `base_link`      | `geometry_msgs/msg/TransformStamped` | 速度積分に基づくodom座標系からbase_linkへのTF        |

#### Parameters

| **Name（Parameter）** | **Type** | **Description**                                                             |
| ---------------------- | -------- | --------------------------------------------------------------------------- |
| `publish_period_ms`     | `int`    | `/localization/odom`およびTFをpublishする周期[ms]                            |
| `max_integration_dt`    | `double` | 速度積分を行う最大dt[s]。これを超える間隔の場合は積分をスキップする          |

### map_odom_tf_node

#### Input

| **Name（Topic / TF）**   | **Type**                                        | **Description**                                                       |
| -------------------------- | -------------------------------------------------- | --------------------------------------------------------------------------- |
| `/localization/pose`        | `geometry_msgs/msg/PoseWithCovarianceStamped`       | `ekf_localizer_node`が推定したmap座標系での自己位置。`frame_id`は`map`である必要がある |
| TF: `odom` -> `base_link`     | `geometry_msgs/msg/TransformStamped`                | `odom_tf_node`が配信するTFを参照                                            |

#### Output

| **Name（Topic / TF）** | **Type**                          | **Description**                             |
| ------------------------ | ------------------------------------ | ------------------------------------------------ |
| TF: `map` -> `odom`        | `geometry_msgs/msg/TransformStamped` | map座標系とodom座標系を接続するTF |

#### Parameters

| **Name（Parameter）**   | **Type** | **Description**                                                       |
| ------------------------ | -------- | --------------------------------------------------------------------- |
| `publish_period_ms`       | `int`    | `map` -> `odom` のTFをpublishする周期[ms]                                |
| `stale_warn_timeout_s`    | `double` | `/localization/pose`の最終更新からこの時間[s]以上経過した場合に警告を出力するまでの閾値 |
