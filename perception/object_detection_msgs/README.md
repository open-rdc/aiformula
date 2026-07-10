# object_detection_msgs

## Package overview

[`object_detector`](../object_detector)が検出した障害物情報のメッセージ定義パッケージです。[`local_planner`](../../planning/local_planner)の回避計画で使用されます。

## Messages

### `ObjectInfo.msg`

検出された1つの障害物を表します。

| **Field** | **Type**  | **Description**             |
| ----------- | ----------- | ------------------------------- |
| `x`           | `float32`   | 障害物中心のx座標                  |
| `y`           | `float32`   | 障害物中心のy座標                  |
| `width`       | `float32`   | 障害物の幅（バウンディングボックス由来） |
| `id`          | `uint8`     | 障害物ID                          |

### `ObjectInfoArray.msg`

1周期分の検出結果一覧を表します。

| **Field** | **Type**            | **Description**       |
| ----------- | --------------------- | -------------------------- |
| `header`      | `std_msgs/Header`      | ヘッダ（座標系・タイムスタンプ） |
| `objects`      | `ObjectInfo[]`         | 検出された障害物の一覧（0件の場合あり） |

