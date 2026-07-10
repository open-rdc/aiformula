# vectormap_server

## Package overview

OSM形式（lanelet2ライクなXML）のベクトルマップファイルをパースし、[`vectormap_msgs/msg/VectorMap`](../vectormap_msgs)として配信するノードです。起動時に一度だけマップを読み込み、latched（`transient_local`）QoSで配信するため、後から起動したノードもマップを取得できます。あわせて`earth`座標系からマップの`frame_id`への静的TFと、RViz可視化用の`MarkerArray`を配信します。

## Nodes

### vectormap_server_node

#### Input

このノードに購読するトピックはありません。

#### Output

| **Name（Topic）**       | **Type**                                | **QoS**                    | **Description**                                               |
| --------------------------- | ------------------------------------------ | ----------------------------- | ------------------------------------------------------------------ |
| `vector_map`                  | `vectormap_msgs/msg/VectorMap`             | depth 1, `transient_local`     | パースしたベクトルマップ。起動時に一度だけ配信                          |
| `vector_map/visualize`         | `visualization_msgs/msg/MarkerArray`       | depth 1, `transient_local`     | マップの線分をRViz上で可視化するためのマーカー。起動時に一度だけ配信          |
| TF: `earth` -> `<map frame_id>` | `geometry_msgs/msg/TransformStamped`      | -                              | マップファイルの`frame_id`（例: `map`）への静的TF                        |

#### Parameters

| **Name（Parameter）** | **Type** | **Description**                                                                                  |
| ------------------------- | -------- | ------------------------------------------------------------------------------------------------------ |
| `map_path`                  | `string` | 読み込むOSMファイル名。`<vectormap_serverのshareディレクトリ>/config/<map_path>`から解決される               |
