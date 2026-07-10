# steered_drive_msg

## Package overview

操舵角と速度からなる走行コマンドのメッセージ定義パッケージです。[`chassis_driver`](../chassis_driver)の`cmd_vel`トピックや、[`control/controller`](../../control/controller)・[`control/trajectory_follower`](../../control/trajectory_follower)が送信する走行指令に用いられます。

## Messages

### `SteeredDrive.msg`

| **Field**          | **Type**  | **Description**                     |
| --------------------- | ----------- | -------------------------------------- |
| `steering_angle`        | `float64`   | 目標操舵角[rad]                        |
| `velocity`               | `float64`   | 目標速度[m/s]                          |

