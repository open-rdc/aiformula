# vectormap_msgs

## Package overview

車線グラフ形式のベクトルマップ（lanelet2ライクな形式）を表現するためのメッセージ定義パッケージです。地物（線分・レーン・エリア）はIDで相互参照するフラットな構造になっており、[`vectormap_server`](../vectormap_server)が配信し、[`pose_estimater`](../../localization/pose_estimater)や[`mission_planner`](../../planning/mission_planner)が利用します。

## Messages

### `LineString.msg`

1本のポリライン（境界線・中心線・仮想線・停止線など）を表します。

| **Field**        | **Type**              | **Description**                                                     |
| ------------------- | ------------------------ | -------------------------------------------------------------------- |
| `id`                  | `uint64`                 | 識別子                                                                |
| `line_type`            | `uint8`                  | 線種（`TYPE_UNKNOWN` / `TYPE_LINE_THIN` / `TYPE_VIRTUAL_LINE` / `TYPE_STOP_LINE`） |
| `line_subtype`          | `uint8`                  | 線のサブタイプ（`SUBTYPE_UNKNOWN` / `SUBTYPE_SOLID` / `SUBTYPE_DASHED` / `SUBTYPE_ROAD_BORDER` / `SUBTYPE_STOP_LINE` / `SUBTYPE_VIRTUAL_LINE`） |
| `marking_type`          | `uint8`                  | 路面標示タイプ（`MARKING_UNKNOWN` / `MARKING_SOLID` / `MARKING_DASHED` / `MARKING_VIRTUAL`） |
| `is_observable`         | `bool`                   | センサから観測可能かどうか                                              |
| `points`                | `geometry_msgs/Point[]`  | 線を構成する点列                                                       |

### `Lanelet.msg`

1つのレーン区間を表します。左右境界線・中心線はIDで参照します。

| **Field**       | **Type**  | **Description**                                                     |
| ------------------ | ----------- | ---------------------------------------------------------------------- |
| `id`                 | `uint64`    | 識別子                                                                  |
| `subtype`            | `uint8`     | レーン種別（`SUBTYPE_UNKNOWN` / `SUBTYPE_ROAD` / `SUBTYPE_INTERSECTION`） |
| `is_virtual`          | `bool`      | 仮想レーンかどうか                                                        |
| `left_line_id`        | `uint64`    | 左境界線（`LineString`）のID                                             |
| `right_line_id`       | `uint64`    | 右境界線（`LineString`）のID                                             |
| `centerline_id`       | `uint64`    | 中心線（`LineString`）のID                                              |

### `LaneConnection.msg`

レーングラフの有向エッジ（`Lanelet`間の接続関係）を表します。経路計画に使用されます。

| **Field**         | **Type**  | **Description**                                                                                     |
| -------------------- | ----------- | ------------------------------------------------------------------------------------------------------- |
| `id`                   | `uint64`    | 識別子                                                                                                    |
| `from_lanelet_id`       | `uint64`    | 接続元`Lanelet`のID                                                                                       |
| `to_lanelet_id`         | `uint64`    | 接続先`Lanelet`のID                                                                                       |
| `turn_direction`        | `uint8`     | 進行方向（`TURN_UNKNOWN` / `TURN_STRAIGHT` / `TURN_LEFT` / `TURN_RIGHT` / `TURN_MERGE` / `TURN_BRANCH` / `TURN_U_TURN`） |
| `cost`                  | `float64`   | 経路コスト                                                                                                 |

### `MapArea.msg`

横断歩道など、外周線で囲まれたエリアを表します。

| **Field**        | **Type** | **Description**                                    |
| ------------------- | ---------- | ----------------------------------------------------- |
| `id`                  | `uint64`   | 識別子                                                  |
| `subtype`             | `uint8`    | エリア種別（`SUBTYPE_UNKNOWN` / `SUBTYPE_CROSSWALK`）    |
| `outer_line_id`        | `uint64`   | 外周線（`LineString`）のID                                |

### `VectorMap.msg`

マップ全体を表すトップレベルメッセージです。

| **Field**          | **Type**                            | **Description**            |
| --------------------- | -------------------------------------- | ------------------------------- |
| `header`                | `std_msgs/Header`                     | ヘッダ                          |
| `map_id`                | `string`                              | マップID                        |
| `map_version`           | `string`                              | マップバージョン                  |
| `line_strings`          | `LineString[]`                        | 線分の一覧                       |
| `lanelets`              | `Lanelet[]`                           | レーン区間の一覧                  |
| `lane_connections`      | `LaneConnection[]`                    | レーングラフの接続関係の一覧          |
| `areas`                 | `MapArea[]`                           | エリア（横断歩道等）の一覧            |

