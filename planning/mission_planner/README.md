# mission_planner

## Package overview

車線グラフ（[`vectormap_msgs`](../../map/vectormap_msgs)の`Lanelet`/`LaneConnection`）に基づいてグローバル経路を生成するノードです。[`vectormap_server`](../../map/vectormap_server)から得たベクトルマップと自己位置から開始レーンを選び、`/planning/nav_cmd`（交差点での進路指定）や`/flag`（車線変更）に応じてグラフを探索し、中心線を接続・リサンプルしたグローバル経路を[`local_planner`](../local_planner)へ配信します。

## Nodes

### mission_planner_node

#### Input

| **Name（Topic）**    | **Type**                                      | **Description**                                                          |
| ------------------------ | -------------------------------------------------- | -------------------------------------------------------------------------------- |
| `/vector_map`               | `vectormap_msgs/msg/VectorMap`                    | 車線グラフを含むベクトルマップ（latched、一度だけ処理してレーン検索テーブルを構築）    |
| `/localization/pose`         | `geometry_msgs/msg/PoseWithCovarianceStamped`      | 自己位置。開始レーンの選定・経路逸脱判定・逸脱時の再構築に使用                          |
| `/planning/nav_cmd`           | `std_msgs/msg/String`                             | 交差点での進路指定（`"straight"` / `"left"` / `"right"`）。受信すると経路を再構築       |
| `/flag`                      | `std_msgs/msg/Empty`                              | 車線変更要求。受信すると隣接レーンを起点に経路を再構築                                  |

#### Output

| **Name（Topic）**     | **Type**              | **Description**       |
| -------------------------- | ------------------------ | -------------------------- |
| `/planner/global_path`       | `nav_msgs/msg/Path`       | 生成されたグローバル経路      |

#### Parameters

| **Name（Parameter）**                       | **Type**   | **Description**                                                                          |
| ------------------------------------------------ | ------------ | ------------------------------------------------------------------------------------------- |
| `update_period_ms`                                  | `int`        | 経路の維持・再配信ループの周期[ms]                                                              |
| `default_nav_cmd`                                    | `string`     | `/planning/nav_cmd`受信前に用いる初期進路指定（`"straight"` / `"left"` / `"right"`）              |
| `nav_cmd_fallback_order`                             | `string[]`   | 交差点で現在の進路指定に合う接続が無い場合に順に試すフォールバック順序                                    |
| `global_path_resample_interval_m`                    | `double`     | 連結した中心線をリサンプルする間隔[m]                                                             |
| `max_centerline_connection_gap_m`                     | `double`     | レーン間の中心線を連結する際に許容する最大ギャップ[m]（超過するとエラー）                                   |
| `off_route_distance_threshold_m`                      | `double`     | 現在の経路の中心線からこの距離[m]以上離れると経路逸脱とみなし再構築する                                     |
| `route_lookahead_lanelet_count`                       | `int`        | 経路構築時に先読みするレーン数（最小3）                                                            |
| `start_lanelet_yaw_threshold_rad`                     | `double`     | 開始レーン選定時に許容する自己位置ヨー角と中心線方向の差の閾値[rad]                                          |
| `start_pose_position_variance_threshold`              | `double`     | 初期経路構築を許可する自己位置の位置共分散の閾値（これを下回るまで経路構築を待つ）                             |
