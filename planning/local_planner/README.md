# local_planner

## Package overview

pluginlibベースのローカル経路計画サーバーです。`local_planner_server_node`が[`mission_planner`](../mission_planner)から得たグローバル経路を、プラグイン（既定はFrenet座標系ベースの`FrenetPlannerPlugin`）に渡してローカル経路を計算し、[`trajectory_follower`](../../control/trajectory_follower)へ配信します。[`object_detector`](../../perception/object_detector)が検出した障害物の静的回避にも対応します。

## Nodes

### local_planner_server_node

#### Input

| **Name（Topic）**       | **Type**                                       | **Description**                                             |
| --------------------------- | -------------------------------------------------- | ------------------------------------------------------------------ |
| `/planner/global_path`        | `nav_msgs/msg/Path`                               | `mission_planner`から得られるグローバル経路（中心線）                    |
| `/localization/pose`          | `geometry_msgs/msg/PoseWithCovarianceStamped`      | 自己位置。計画の起点として使用                                          |
| `/vectornav/velocity_body`     | `geometry_msgs/msg/TwistWithCovarianceStamped`     | 現在の車体速度。回避時の躍度制限付き横移動距離の算出に使用                      |
| `/perception/objects`          | `object_detection_msgs/msg/ObjectInfoArray`        | 検出された障害物一覧。静的障害物回避に使用                                  |

#### Output

| **Name（Topic）**    | **Type**              | **Description**             |
| ------------------------ | ------------------------ | -------------------------------- |
| `/planner/local_path`      | `nav_msgs/msg/Path`       | 計画されたローカル経路               |

#### Parameters

| **Name（Parameter）**                        | **Type** | **Description**                                                                   |
| ------------------------------------------------ | -------- | ---------------------------------------------------------------------------------------- |
| `update_period_ms`                                  | `int`    | 計画ループの周期[ms]                                                                        |
| `local_planner_plugin`                              | `string` | 使用するプラグインのpluginlibルックアップ名（既定: `"local_planner::FrenetPlannerPlugin"`）        |

## Plugins

### FrenetPlannerPlugin（`local_planner::FrenetPlannerPlugin`）

自己位置をグローバル経路上のFrenet座標`(s, d)`に射影し、`local_path_horizon_m`分だけ先読みした候補経路を生成します。障害物が前方検知範囲内にある場合は躍度制限付きの横移動（イーズイン・イーズアウト）で回避候補を追加し、衝突判定でふるいにかけた上でコスト最小の経路を選択、直交座標に変換して配信します。

| **Name（Parameter）**                       | **Type** | **Description**                                                     |
| ----------------------------------------------- | -------- | --------------------------------------------------------------------- |
| `local_path_horizon_m`                             | `double` | ローカル経路を計画する先読み距離[m]（現在の`s`からの距離）                       |
| `local_path_resample_interval_m`                   | `double` | ローカル経路生成時の縦方向サンプリング間隔[m]                                    |
| `max_centerline_connection_gap_m`                  | `double` | グローバル経路の始点・終点間ギャップから閉ループ経路かどうかを判定する閾値[m]              |
| `vehicle_width_m`                                   | `double` | 車体幅[m]。衝突判定・回避時の横方向マージン計算に使用                              |
| `avoidance_detection_forward_distance_m`            | `double` | 障害物回避判定の前方検知距離[m]                                                |
| `avoidance_hard_margin_m`                           | `double` | 車体エンベロープに加える固定の横方向マージン[m]                                    |
| `avoidance_soft_margin_m`                           | `double` | 障害物検知範囲・回避移動量に加える追加の横方向マージン[m]                             |
| `envelope_buffer_margin_m`                          | `double` | 車体エンベロープにさらに加えるバッファマージン[m]                                    |
| `avoidance_lateral_jerk_mps3`                       | `double` | 回避時の横移動に許容する最大躍度[m/s^3]                                          |
| `avoidance_min_velocity_mps`                        | `double` | 回避移動距離算出時に用いる最小速度[m/s]（低速時のフロア）                             |
| `max_avoidance_shift_m`                             | `double` | 回避のための横移動量の上限[m]                                                   |
| `frenet_collision_check_margin_m`                   | `double` | 候補経路の衝突判定に追加する安全マージン[m]                                        |
| `frenet_weight_lateral_offset`                      | `double` | 候補経路の最終横オフセットに対するコスト重み                                        |
| `frenet_weight_lateral_change`                      | `double` | 候補経路の横方向変化量（蛇行）に対するコスト重み                                     |
| `frenet_weight_avoidance_shift`                     | `double` | 回避移動量そのものに対するコスト重み                                              |
