# controller

## Package overview

ジョイスティックからの入力を解釈し、手動走行コマンドやイベント（自動運転切り替え、リスタート、車線変更フラグ、交差点でのナビゲーションコマンド）を配信するteleopノードです。`controller_node`として[`main_executor`](../../main_executor)に組み込まれて動作します。

## Nodes

### controller_node

#### Input

| **Name（Topic）** | **Type**             | **Description** |
| -------------------- | ----------------------- | ------------------ |
| `joy`                 | `sensor_msgs/msg/Joy`    | ジョイスティックの生入力  |

#### Output

| **Name（Topic）**   | **Type**                            | **Description**                                                          |
| ---------------------- | -------------------------------------- | ------------------------------------------------------------------------- |
| `cmd_vel`                | `steered_drive_msg/msg/SteeredDrive`   | 手動走行時の速度・操舵指令                                |
| `restart`                | `std_msgs/msg/Empty`                  | 駆動系リスタート信号（Optionsボタン）                                     |
| `autonomous`              | `std_msgs/msg/Bool`                   | 自律／手動の切り替え（Shareボタン）                                |
| `/planning/nav_cmd`        | `std_msgs/msg/String`                 | 交差点での進路選択コマンド（`"left"` / `"right"` / `"straight"`、L1/R1/L2） |
| `/flag`                   | `std_msgs/msg/Empty`                  | 車線変更フラグ（Crossボタン）                                          |

#### Parameters

| **Name（Parameter）** | **Type** | **Description**                                                    |
| ------------------------- | -------- | ---------------------------------------------------------------------- |
| `linear_max_vel`            | `double` | 手動走行時の最大速度[m/s]（ジョイスティック左スティックに乗算）           |
| `steering_max.pos`          | `double` | 手動走行時の最大操舵角[deg]（`/**`共通パラメータ、内部でradに変換し右スティックに乗算） |
