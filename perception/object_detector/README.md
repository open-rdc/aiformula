# object_detector

## Package overview

ZEDカメラの点群から障害物を検出するノードです。地面点除去・ボクセルダウンサンプリング・ユークリッドクラスタリング（PCL）によって点群をクラスタ化し、各クラスタの重心・幅を[`object_detection_msgs`](../object_detection_msgs)として配信します。出力は[`local_planner`](../../planning/local_planner)の障害物回避に利用されます。

## Nodes

### object_detector_node

#### Input

| **Name（Topic）**            | **Type**                        | **Description**                    |
| --------------------------------- | ---------------------------------- | ---------------------------------------- |
| `/zed/zed_node/point_cloud`         | `sensor_msgs/msg/PointCloud2`     | ZEDカメラから得られる生の点群               |

（`base_link`・`map`へのTF変換にTFバッファを使用します。）

#### Output

| **Name（Topic）**              | **Type**                                       | **Description**                                |
| ------------------------------------ | -------------------------------------------------- | ------------------------------------------------------ |
| `/perception/objects`                  | `object_detection_msgs/msg/ObjectInfoArray`        | 検出された障害物クラスタの一覧（`map`座標系）              |
| `/perception/objects_visualize`        | `visualization_msgs/msg/MarkerArray`               | 検出された障害物のRViz可視化用マーカー（CYLINDER、`map`座標系） |

#### Parameters

| **Name（Parameter）** | **Type** | **Description**                                                  |
| ------------------------- | -------- | ---------------------------------------------------------------------- |
| `ground_z_threshold_m`      | `double` | 地面除去用のz方向PassThroughフィルタの閾値[m]                             |
| `voxel_leaf_size_m`         | `double` | ボクセルグリッドダウンサンプリングのリーフサイズ[m]                          |
| `cluster_tolerance_m`       | `double` | ユークリッドクラスタリングの距離許容値[m]                                    |
| `min_cluster_size`          | `int`    | クラスタとして認識する最小点数                                              |
| `max_cluster_size`          | `int`    | クラスタとして認識する最大点数                                              |
| `marker_height_m`           | `double` | 可視化マーカー（円柱）の高さ[m]                                             |
