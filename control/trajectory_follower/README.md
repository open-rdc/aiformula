# trajectory_follower

## Package overview

pluginlibベースのプラグインアーキテクチャを持つ経路追従制御サーバーです。`controller_server_node`が[`local_planner`](../../planning/local_planner)から得たローカル経路を、設定されたプラグイン（Pure PursuitまたはPID+MPC）に渡して速度・操舵指令を計算し、`/cmd_vel`として配信します。プラグインの切り替えはパラメータのみで行えます。

## Nodes

### controller_server_node

制御周期ごとにタイマーで駆動され、自律走行が有効かつローカル経路が存在する場合のみ指令を計算・配信します。経路が`map`座標系の場合は自己位置を用いて`base_link`座標系に変換してからプラグインに渡します。

#### Input

| **Name（Topic）**       | **Type**                                       | **Description**                                                            |
| -------------------------- | -------------------------------------------------- | -------------------------------------------------------------------------------- |
| `/planner/local_path`        | `nav_msgs/msg/Path`                               | 追従対象のローカル経路（`map`または`base_link`座標系）                          |
| `/localization/pose`         | `geometry_msgs/msg/PoseWithCovarianceStamped`      | 自己位置（`map`座標系のみ許容）。経路を`base_link`へ変換する際に使用             |
| `/vectornav/velocity_body`   | `geometry_msgs/msg/TwistWithCovarianceStamped`     | 現在の車体速度（`twist.twist.linear.x`）。プラグインへ渡す速度フィードバック    |
| `/autonomous`                | `std_msgs/msg/Bool`                               | 自律走行フラグ。falseの場合は指令を計算・配信しない                          |

#### Output

| **Name（Topic）**              | **Type**                              | **Description**                                        |
| ---------------------------------- | ---------------------------------------- | ------------------------------------------------------------ |
| `/cmd_vel`                           | `steered_drive_msg/msg/SteeredDrive`     | プラグインが計算した速度・操舵指令                             |
| `/vectormap_control/target_pose`      | `geometry_msgs/msg/PoseStamped`          | プラグインが選択した目標（先読み）姿勢（`base_link`座標系、可視化用） |

#### Parameters

| **Name（Parameter）** | **Type** | **Description**                                                                 |
| ------------------------- | -------- | ------------------------------------------------------------------------------- |
| `control_period_ms`         | `int`    | 制御ループの周期[ms]                                                            |
| `controller_plugin`         | `string` | 使用するプラグインのpluginlibルックアップ名（例: `"trajectory_follower::PurePursuitPlugin"`） |

## Plugins

pluginlibインタフェース`trajectory_follower::ControllerPlugin`を実装するプラグインを、`controller_plugin`パラメータで切り替えて使用します。

### PurePursuitPlugin（`trajectory_follower::PurePursuitPlugin`）

Pure Pursuit制御。経路上の先読み点を探索し、幾何学的なPure Pursuitの式で操舵角を、先読み点までの距離に応じて速度を計算します。

| **Name（Parameter）**       | **Type** | **Description**                                                    |
| -------------------------- | -------- | -------------------------------------------------------------------- |
| `linear_max.vel`             | `double` | 最大速度[m/s]（`/**`共通パラメータ）。先読み点距離に応じて指令速度をスケーリング |
| `pure_pursuit.lookahead_distance` | `double` | 先読み距離[m]。先読み点の選択と操舵角式の分母、速度スケーリングの正規化に使用 |
| `pure_pursuit.steered_gain`  | `double` | Pure Pursuit操舵角に乗じるゲイン                                       |
| `wheelbase`                  | `double` | ホイールベース[m]（`/**`共通パラメータ）。操舵角式に使用                    |
| `steering_max.pos`           | `double` | 最大操舵角[deg]（`/**`共通パラメータ、内部でradに変換）。指令操舵角をクランプ |

### PidMpcPlugin（`trajectory_follower::PidMpcPlugin`）

縦方向をPID、横方向を線形MPCで制御するプラグイン。縦方向は経路曲率による横加速度制限・終端までの制動距離から目標速度を求めPID＋フィードフォワードで追従し、横方向は`[横偏差, 方位偏差, 操舵角]`の3状態誤差モデルに対する線形MPCで操舵角を求めます。

| **Name（Parameter）**                     | **Type** | **Description**                                              |
| ------------------------------------------ | -------- | ------------------------------------------------------------ |
| `control_period_ms`                          | `int`    | 制御周期[ms]（サーバーと共通）。縦方向PIDの積分周期`dt`に使用     |
| `linear_max.vel`                              | `double` | 最大速度[m/s]（`/**`共通パラメータ）。目標速度の上限           |
| `mpc.longitudinal.a_lat_max`                  | `double` | 許容最大横加速度[m/s^2]。カーブでの目標速度上限に使用            |
| `mpc.longitudinal.a_max`                      | `double` | 最大加速度[m/s^2]                                              |
| `mpc.longitudinal.a_min`                      | `double` | 最大減速度[m/s^2]（負値）。制動距離による目標速度上限にも使用       |
| `mpc.longitudinal.jerk_max`                   | `double` | 最大躍度[m/s^3]（加速度の変化率制限）                             |
| `mpc.longitudinal.kp`                         | `double` | 速度追従PIDの比例ゲイン                                          |
| `mpc.longitudinal.ki`                         | `double` | 速度追従PIDの積分ゲイン                                          |
| `mpc.longitudinal.kd`                         | `double` | 速度追従PIDの微分ゲイン                                          |
| `mpc.longitudinal.lpf_vel_error_gain`         | `double` | 速度偏差に対するローパスフィルタ係数                              |
| `wheelbase`                                    | `double` | ホイールベース[m]（`/**`共通パラメータ）。横方向モデルに使用       |
| `mpc.lateral.steer_tau`                        | `double` | 操舵アクチュエータの一次遅れ時定数[s]                             |
| `steering_max.pos`                             | `double` | 最大操舵角[deg]（`/**`共通パラメータ、内部でradに変換）。MPC出力をクランプ |
| `mpc.lateral.weight_lat_error`                 | `double` | MPCコストの横偏差重み                                            |
| `mpc.lateral.weight_heading_error`             | `double` | MPCコストの方位偏差重み                                          |
| `mpc.lateral.weight_steering_input`            | `double` | MPCコストの操舵入力量に対する重み                                 |
| `mpc.lateral.weight_steer_rate`                | `double` | MPCコストの操舵角変化率に対する重み                               |
| `mpc.lateral.weight_terminal_lat_error`        | `double` | MPCコストのホライズン終端における横偏差重み                        |
| `mpc.lateral.weight_terminal_heading_error`    | `double` | MPCコストのホライズン終端における方位偏差重み                      |
| `mpc.lateral.horizon`                          | `int`    | MPC予測ホライズンのステップ数                                     |
| `mpc.lateral.prediction_dt`                    | `double` | MPC予測モデルの離散化時間刻み[s]                                  |
| `mpc.lateral.min_predict_speed`                | `double` | MPC予測モデルで使用する最小速度[m/s]（低速時の特異点回避）           |
