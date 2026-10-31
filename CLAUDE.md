# aiformula

AI Formula（自律走行レース競技）車両の ROS 2 ワークスペース。

## ビルドと検証

**テストスイートは存在しない**（`colcon test` は lint のみで、大半のパッケージは copyright / cpplint を無効化済み）。
変更後は以下のビルドが通ることを確認し、検証結果を報告すること。

```bash
colcon build --symlink-install --packages-up-to main_executor
```

- 全 C++ ノードは `main_executor` にリンクされるため、変更パッケージ単体の `--packages-select` だけでは
  `main_executor` 側のリンク切れ・API 不整合を検出できない。必ず `main_executor` まで含めてビルドする
- Python パッケージ（`road_detector`）のみの変更は
  `colcon build --symlink-install --packages-select road_detector` で足りる

## アーキテクチャ上の重要な制約

### 全 C++ ノードは 1 プロセスに合成される

`main_executor/src/main.cpp` が全ノードを `MultiThreadedExecutor` に `add_node` する単一プロセス構成。
そのため **ノードクラスは必ず `explicit Xxx(const rclcpp::NodeOptions& options)` コンストラクタを持つこと**。

新しい C++ ノードを追加するときは 3 ヶ所を同時に更新する:

1. `main_executor/src/main.cpp`（include + `make_shared` + `add_node`）
2. `main_executor/package.xml`（`<depend>`）
3. 対象パッケージ側で `ament_auto_add_library(... SHARED)` としてライブラリを公開

Python ノード（`road_detector`）だけは合成対象外で、独立プロセスとして起動する
（rclpy ノードは rclcpp の executor に合成できないため）。

### パラメータは 1 ファイルに集約

全ノードのパラメータは `main_executor/config/main_params.yaml` のみ。パッケージごとの yaml は作らない。

- トップレベルのキー名は **ノード名と完全一致** させる（例: `localization_node:`）。
  一致しないとそのノードにパラメータが渡らない
- `/**:` セクションの値（`linear_max`, `steering_max`, `tread`, `wheelbase`）は全ノード共通で、
  どのノードからも読める
- `launch:` セクションの真偽値が
  起動するハードウェアノードを切り替える。`sim: true` のときは `use_sim_time` が自動で有効化される

### 独自メッセージは `msgs/` 配下に置く

自作の msg パッケージはすべて `src/msgs/` 直下に並べる。ドメイン側のディレクトリ
（`map/`, `perception/`, `control/` など）には msg パッケージを置かない。

```
src/msgs/
├── vectormap_msgs/       # 地図（VectorMap, Lanelet, LineString, LaneConnection, MapArea）
├── object_detection_msgs/# 物体検出（ObjectInfo, ObjectInfoArray）
└── steered_drive_msg/    # 車体指令（SteeredDrive）
```

- パッケージ名・型名はディレクトリ位置に依存しないため、移動しても利用側の
  `#include` や `<depend>` は変更不要
- サブモジュール配下の msg（`socketcan_interface_msg`, `odrive_node`, `vectornav_msgs`）は対象外

### pluginlib

`motion_control`（`controller_plugin`）と `local_planner`（`local_planner_plugin`）は pluginlib 方式。
プラグイン追加時は以下をセットで更新する:

1. 実装クラス（`.cpp` 末尾の `PLUGINLIB_EXPORT_CLASS` マクロを忘れない）
2. `plugins.xml`
3. `CMakeLists.txt` の `pluginlib_export_plugin_description_file`
4. `main_params.yaml` の `*_plugin` に完全修飾名を記載（例: `motion_control::PurePursuitPlugin`）

### ZED SDK はオプショナル依存

`zed_wrapper/CMakeLists.txt` は ZED SDK 未検出時にビルドを **警告のみでスキップ** する。
`main.cpp` 側は `#ifdef ENABLE_ZED` でガードされているため、SDK のない環境でもワークスペース全体はビルドできる。
ZED 関連コードを触るときはこのガード構造を崩さないこと。

## コード規約

- C++17。ビルドは `ament_cmake_auto`（`ament_auto_find_build_dependencies` + `ament_auto_add_library`）。
  依存は `CMakeLists.txt` に直接書かず `package.xml` の `<depend>` に書く
- パッケージ構成: `include/<pkg名>/*.hpp` + `src/*.cpp`、`namespace <pkg名>`。
  `visibility_control.h` を用意し、公開 API に `<PKG>_PUBLIC` マクロを付ける
- ヘッダガードは `#pragma once`
- コメント・パラメータの説明は日本語で可（既存コードに合わせる）
