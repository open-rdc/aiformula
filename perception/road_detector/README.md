# road_detector

## Package overview

YOLOPv2モデルによる車線セグメンテーションを行うPythonノードです。カメラ画像から車線境界線マスクを推論し、[`lane_line_publisher`](../lane_line_publisher)へ配信します。


## Nodes

### road_detector_node

#### Input

| **Name（Topic、パラメータで変更可）** | **Type**                | **Description**                          |
| ---------------------------------------- | -------------------------- | ---------------------------------------------- |
| `/zed/zed_node/rgb/image_rect_color`（既定値） | `sensor_msgs/msg/Image`   | 推論に使用するカメラRGB画像（`input_image_topic`で変更可） |

#### Output

| **Name（Topic、パラメータで変更可）** | **Type**                       | **Description**                                |
| ---------------------------------------- | --------------------------------- | ------------------------------------------------------ |
| `/perception/lane_mask`（既定値）           | `sensor_msgs/msg/Image`（mono8） | 車線セグメンテーションマスク（`output_mask_topic`で変更可）    |

#### Parameters

| **Name（Parameter）**  | **Type / Default**                          | **Description**                                    |
| -------------------------- | ---------------------------------------------- | -------------------------------------------------------- |
| `input_image_topic`           | `string` / `/zed/zed_node/rgb/image_rect_color`  | 入力画像トピック                                             |
| `output_mask_topic`            | `string` / `/perception/lane_mask`               | 出力マスクトピック                                            |
| `capture_width`                | `int` / `640`                                    | 推論前にリサイズする幅[px]                                     |
| `capture_height`               | `int` / `360`                                    | 推論前にリサイズする高さ[px]                                    |
| `visualize`                    | `bool` / `false`                                 | trueの場合、OpenCVウィンドウにマスクを重畳表示する                  |
