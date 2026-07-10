# chassis_driver

## Package overview

[`steered_drive_msg/msg/SteeredDrive`](../steered_drive_msg)（操舵角＋速度）で表される走行コマンドを、実際の車体アクチュエータへの指令に変換するパッケージです。以下の2ノードで構成されます。

* `chassis_driver_node`: 走行コマンドを速度プランナーで平滑化し、キャスター（操舵輪）をODriveの位置制御でケーブル駆動しつつ、左右駆動輪の差動制御によりCAN経由でRPM指令を送信します。また、キャスターのエンコーダからオドメトリを算出して配信します。
* `debug_printer_node`: CANの生データ（RPMフィードバック、ポテンショメータ値など）を人が読める`Int64`トピックにデコードして配信するデバッグ用ノードです。

## Nodes

### chassis_driver_node

走行コマンドをキャスター位置指令・駆動輪RPM指令に変換し、CAN経由でODrive・モータコントローラに送信します。あわせてキャスターのエンコーダからオドメトリを算出します。速度は台形加減速プランナーで平滑化され、操舵はキャスターの向きに応じたケーブル巻き取り量として、駆動輪の左右差はキャスターの向き誤差に対するPID制御として実現されます。緊急停止信号を受信するか`restart`を受けるとモード（`stay`/`cmd`/`stop`）が切り替わります。

#### Input

| **Name（Topic）**            | **Type**                                       | **Description**                                                                 |
| ------------------------------ | -------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `cmd_vel`                        | `steered_drive_msg/msg/SteeredDrive`               | 目標の操舵角・速度からなる走行コマンド                                                |
| `restart`                        | `std_msgs/msg/Empty`                              | モードを`stay`にリセットし、速度プランナー・オドメトリを初期化してODriveをclosed-loop状態に再要求する |
| `can_rx_012`                     | `socketcan_interface_msg/msg/SocketcanIF`          | キャスター向きエンコーダのCAN受信値                                                    |
| `can_rx_013`                     | `socketcan_interface_msg/msg/SocketcanIF`          | キャスター回転エンコーダのCAN受信値（オドメトリ・リール量計算に使用）                    |
| `can_rx_712`                     | `socketcan_interface_msg/msg/SocketcanIF`          | 非常停止信号のCAN受信値。有効時は強制的に`stop`モードへ                               |
| `vectornav/velocity_body`        | `geometry_msgs/msg/TwistWithCovarianceStamped`    | VectorNavから得られる車体速度。操舵プリロード・トルク差指令のスケーリングに使用          |

#### Output

| **Name（Topic）**              | **Type**                              | **Description**                                                     |
| --------------------------------- | ---------------------------------------- | ------------------------------------------------------------------------ |
| `can_tx`                            | `socketcan_interface_msg/msg/SocketcanIF` | 左右駆動輪RPM指令（CAN ID `0x210`）                                       |
| `/odrive_axis0/control_message`      | `odrive_can/msg/ControlMessage`          | キャスター操舵用ODrive軸への位置指令（position control）                  |
| `caster_data`                       | `std_msgs/msg/Float64MultiArray`         | デバッグ用配列 `[目標操舵量, 計測キャスター向き, キャスター回転量]`         |
| `caster_odom`                       | `nav_msgs/msg/Odometry`                  | キャスターのエンコーダから積分した車体オドメトリ（`frame_id: base_link`） |

#### Services

* `/odrive_axis0/request_axis_state`（クライアント）: `odrive_can/srv/AxisState` — `restart`受信時に`axis_requested_state=8`（CLOSED_LOOP_CONTROL）を要求する

#### Parameters

| **Name（Parameter）**   | **Type** | **Description**                                                    |
| -------------------------- | -------- | -------------------------------------------------------------------- |
| `interval_ms`               | `int`    | 指令publishタイマーの周期[ms]（PID制御のサンプリング周期にも使用）    |
| `wheel_radius`              | `double` | 駆動輪半径[m]（RPM算出に使用）                                       |
| `tread`                     | `double` | 左右駆動輪間のトレッド幅[m]（差動駆動の運動学に使用）                 |
| `wheelbase`                 | `double` | ホイールベース[m]（オドメトリのyaw変化量算出に使用）                  |
| `reduction_ratio`           | `double` | 減速比（RPM出力へのスケーリングに使用）                              |
| `reverse_left_flag`         | `bool`   | 左輪RPMの符号反転フラグ                                              |
| `reverse_right_flag`        | `bool`   | 右輪RPMの符号反転フラグ                                              |
| `caster.max_count`          | `int`    | キャスターエンコーダの1回転あたりカウント数                          |
| `caster.gear_ratio`         | `double` | キャスター回転エンコーダのギア比                                     |
| `caster.wheel_radius`       | `double` | キャスター（前輪）半径[m]（オドメトリの移動量算出に使用）             |
| `caster.reel_radius`        | `double` | 巻き取り長からモータ位置指令[rad]へ変換する際のリール半径[m]           |
| `caster.steering_radius`    | `double` | 操舵時の巻き取り長算出に用いる半径[m]                                 |
| `caster.preload_length`     | `double` | 直進時に用いる固定巻き取り長[m]                                       |
| `caster.preload_gain`       | `double` | `sin(caster_orientation) * 車体速度^2`に乗じる巻き取り量のゲイン      |
| `linear_max.vel`            | `double` | 速度プランナーの最大速度[m/s]（`/**`共通パラメータ）                  |
| `linear_max.acc`            | `double` | 速度プランナーの最大加速度[m/s^2]（`/**`共通パラメータ）              |
| `steering_max.pos`          | `double` | 最大操舵角[deg]（`/**`共通パラメータ、内部でradに変換）               |
| `drive_pid.p_gain`          | `double` | 駆動輪トルク差PIDの比例ゲイン                                        |
| `drive_pid.i_gain`          | `double` | 駆動輪トルク差PIDの積分ゲイン                                        |
| `drive_pid.d_gain`          | `double` | 駆動輪トルク差PIDの微分ゲイン                                        |

`tread` / `wheelbase` / `linear_max.*` / `steering_max.pos` は[`main_executor/config/main_params.yaml`](../../main_executor/config/main_params.yaml)の`/**`（全ノード共通）セクションで定義されています。

### debug_printer_node

CANの生フレームを人が読める形式にデコードして配信する、デバッグ専用ノードです。パラメータはありません。

#### Input

| **Name（Topic）** | **Type**                                  | **Description**                                              |
| -------------------- | -------------------------------------------- | ------------------------------------------------------------------ |
| `can_rx_711`          | `socketcan_interface_msg/msg/SocketcanIF`    | モータコントローラからのRPMフィードバック（byte0-3:左輪, byte4-7:右輪） |
| `can_rx_11`           | `socketcan_interface_msg/msg/SocketcanIF`    | ポテンショメータの読み値                                            |
| `can_tx`              | `socketcan_interface_msg/msg/SocketcanIF`    | `chassis_driver_node`が送信するRPM指令（CAN ID `0x210`）の監視     |

#### Output

| **Name（Topic）** | **Type**              | **Description**             |
| -------------------- | ------------------------ | -------------------------------- |
| `left_rpm_rx`         | `std_msgs/msg/Int64`     | 左輪RPMフィードバック（デコード後） |
| `right_rpm_rx`        | `std_msgs/msg/Int64`     | 右輪RPMフィードバック（デコード後） |
| `left_rpm_tx`         | `std_msgs/msg/Int64`     | 左輪RPM指令（デコード後）           |
| `right_rpm_tx`        | `std_msgs/msg/Int64`     | 右輪RPM指令（デコード後）           |
| `potentio`            | `std_msgs/msg/Int64`     | ポテンショメータの読み値（デコード後） |
