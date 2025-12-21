# path_tracker (MPC パス追従パッケージ)

このパッケージは、差動二輪ロボット（およびアッカーマンモデル相当の運動特性を持つロボット）向けのモデル予測制御 (MPC) を提供します。
幾何学的な運動学モデルと、摩擦や慣性を考慮した物理ダイナミクスモデルの両方をサポートしています。

## 特徴
- **CasADi + IPOPT**: 高速かつ強力な最適化ソルバーを使用。
- **Right+ 統一座標系**: 内部モデル、指令値、プロッターをすべて「右旋回正」で統一し、位相の混乱を解消。
- **極低振動制御**: 加速度および転舵速度に対する強力なペナルティと、フィードバック値へのローパスフィルタにより、滑らかな走行を実現。
- **動的モデル (Dynamic Model)**: ロボットの質量、慣性、直進・旋回抵抗を考慮した高精度な予測。

## セットアップ
ビルドには CasADi が必要です。

```bash
colcon build --packages-select path_tracker --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 使い方

### MPC ノードの起動

```bash
ros2 run path_tracker mpc_node --ros-args \
  --remap /cmd_vel:=/cmd_vel_mpc \
  --remap /e2e_planner/path:=/gnss_path \
  -p use_dynamic_model:=true
```

#### 主要なパラメータ
- `use_dynamic_model` (bool): `true` で物理モデルを使用、`false` で幾何モデル（シンプル）を使用。
- `Kv`, `Cv`: 速度系の物理定数（推定器で得られた値を設定）。
- `Kw`, `Cw`: 旋回系の物理定数（推定器で得られた値を設定）。
- `max_accel`: 最大加速度 [m/s^2] (推奨: 0.3〜0.5)。
- `max_delta_rate`: 最大転舵速度 [rad/s] (推奨: 1.0)。

---

## 支援ツール

### 1. 性能比較プロッター (`compare_mpc.py`)
指令値（Bag内/MPC）と実測値（IMU/Odom）をリアルタイムで比較表示します。
- `Bag Cmd`: Bagファイルに記録されている元の司令。
- `MPC Cmd`: 本ノードが出力している現在の司令。
- `Real Odom/Angular`: 実際のロボットの動き。

```bash
ros2 run path_tracker compare_mpc.py
```

### 2. パラメータ推定器 (`estimate_params.py`)
Bagファイルの走行データから、ロボットの $K, C$ 定数を自動計算します。
1. `rosbag play` でデータを流す。
2. スクリプトを実行し、しばらく待ってから Enter を押す。
3. 推定された値を MPC のパラメータに反映させる。

```bash
ros2 run path_tracker estimate_params.py
```

---

## パラメータ調整のコツ
- **振動を抑えたい**: `casadi_solver.cpp` の `w_delta_rate` (10000〜) を大きくする。
- **追従を鋭くしたい**: `casadi_solver.cpp` の `w_pos` (100〜) を大きくする。
- **速度の跳ねを抑えたい**: `mpc_node.cpp` の `latest_v_` に対するフィルタ（alpha）を小さくし、`w_accel` を大きくする。
