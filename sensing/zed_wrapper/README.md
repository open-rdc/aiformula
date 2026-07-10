# zed_wrapper

## Package overview

Stereolabs ZED SDKをラップし、ステレオカメラの映像・点群をROS2メッセージとして配信するノードです。ZED SDKが利用可能な環境でのみビルドされ（`ENABLE_ZED`）、[`main_executor`](../../main_executor)からは`launch.zed`パラメータが真の場合のみ起動されます。

## Nodes

### zed_wrapper_node

#### Input

このノードに購読するトピックはありません。

#### Output

| **Name（Topic）**                | **Type**                        | **Description**                                        |
| ------------------------------------- | ----------------------------------- | -------------------------------------------------------------- |
| `/zed/zed_node/rgb/image_rect_color`    | `sensor_msgs/msg/Image`（bgra8）    | 左カメラの整流済みカラー画像（640x360）                            |
| `/zed/zed_node/point_cloud`             | `sensor_msgs/msg/PointCloud2`      | カラー付き3次元点群（x, y, z, rgba）                              |
| `/zed/zed_node/rgb/camera_info`         | `sensor_msgs/msg/CameraInfo`       | 左カメラの内部パラメータ（ZED SDKのキャリブレーション値から構築）        |

#### Parameters

| **Name（Parameter）**   | **Type** | **Description**                                                                                      |
| --------------------------- | -------- | ------------------------------------------------------------------------------------------------------------ |
| `grab_fps`                     | `int`    | カメラのフレームレート[fps]。ROS側のpublishタイマー周期もこれに合わせる                                              |
| `resolution`                   | `string` | 撮影解像度（`"HD1200"` / `"HD1080"` / `"SVGA"`）                                                                |
| `depth_mode`                   | `string` | 深度推定モード（`"NONE"` / `"PERFORMANCE"` / `"QUALITY"` / `"ULTRA"` / `"NEURAL"` / `"NEURAL_PLUS"`）             |
| `confidence_threshold`          | `int`    | 深度の信頼度しきい値                                                                                          |
| `serial_number`                 | `int`    | 使用するZEDカメラのシリアル番号（`0`の場合は検出された最初のカメラを使用）                                              |
