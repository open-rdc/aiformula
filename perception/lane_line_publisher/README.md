# lane_line_publisher

## Package overview

[`road_detector`](../road_detector)が出力する車線セグメンテーションマスクを地面平面へ投影し、`base_link`座標系の3次元車線境界線点群として配信するノードです。カメラの内部パラメータ・車体への外部パラメータからレイキャスティング用のルックアップテーブルをあらかじめ構築し、マスク画像の細線化（thinning）結果を投影します。出力は[`pose_estimater`](../../localization/pose_estimater)のICP観測点として利用されます。

## Nodes

### lane_line_publisher_node

#### Input

| **Name（Topic）**       | **Type**                  | **Description**                                  |
| --------------------------- | ---------------------------- | ------------------------------------------------------ |
| `/perception/lane_mask`       | `sensor_msgs/msg/Image`（mono8） | `road_detector`が出力する車線セグメンテーションマスク       |

#### Output

| **Name（Topic）**            | **Type**                              | **Description**                                       |
| --------------------------------- | ---------------------------------------- | ------------------------------------------------------------ |
| `/perception/lane_line_points`      | `sensor_msgs/msg/PointCloud2`           | 地面平面へ投影した車線境界線点群（`base_link`座標系）             |
| `/perception/lane_line`             | `visualization_msgs/msg/MarkerArray`    | 車線境界線点群のRViz可視化用マーカー（POINTS）                     |

#### Parameters

| **Name（Parameter）**                        | **Type** | **Description**                                            |
| ------------------------------------------------ | -------- | ---------------------------------------------------------------- |
| `mask_threshold`                                    | `int`    | マスク画像の二値化閾値                                              |
| `pixel_step`                                        | `int`    | 投影ルックアップテーブルを構築する際のピクセル間隔（間引き幅）             |
| `max_observed_points`                               | `int`    | 配信する車線点群の最大点数                                            |
| `camera.fx` / `camera.fy` / `camera.cx` / `camera.cy` | `double` | カメラ内部パラメータ                                                |
| `ground_plane_z_base`                               | `double` | `base_link`座標系における地面平面の高さ[m]                            |
| `min_ground_intersection_distance`                  | `double` | 地面との交点として有効とみなす最小距離[m]                               |
| `max_ground_intersection_distance`                  | `double` | 地面との交点として有効とみなす最大距離[m]                               |
| `camera_to_base.x` / `.y` / `.z`                     | `double` | カメラから`base_link`への並進外部パラメータ[m]                        |
| `camera_to_base.roll` / `.pitch` / `.yaw`            | `double` | カメラから`base_link`への回転外部パラメータ[rad]                       |
