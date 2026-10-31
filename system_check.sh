#!/usr/bin/env bash
#
# aiformula 実機システム状態チェック
#
# main_exec.launch.py を起動した状態で実行し、以下をまとめて確認する。
#
#   1. ROS 2 実行環境
#   2. ノードの生存
#   3. トピックの出版状況と周波数
#   4. TF
#   5. CAN バス
#   6. もう一方のマシン (Jetson / DellPC) との接続
#
# 実機専用。main_params.yaml の launch.sim が true のときは非対応として終了する。
#
#   使い方:
#     ./system_check.sh
#

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
# 設定
#   このスクリプトを置くマシンに合わせて書き換える。
# ---------------------------------------------------------------------------

# 実行ロール
#   "jetson" : 知覚 / 自己位置 / 計画 / TF を確認する
#   "dell"   : chassis_driver / CAN / ODrive を確認する
readonly ROLE="jetson"

# 各マシンの固定 IP
readonly JETSON_IP="192.168.10.10"
readonly DELL_PC_IP="192.168.10.20"

# 周波数の計測時間 [秒]
readonly MEASURE_DURATION=10.0

# ロールから自機と相手機を決める
if [[ "$ROLE" == "dell" ]]; then
  readonly OWN_NAME="DellPC"  OWN_IP="$DELL_PC_IP"
  readonly PEER_NAME="Jetson" PEER_IP="$JETSON_IP"
else
  readonly OWN_NAME="Jetson"  OWN_IP="$JETSON_IP"
  readonly PEER_NAME="DellPC" PEER_IP="$DELL_PC_IP"
fi

# ---------------------------------------------------------------------------
# ログ出力
# ---------------------------------------------------------------------------
OK_COUNT=0
NG_COUNT=0
WARN_COUNT=0
SKIP_COUNT=0

if [[ -t 1 ]]; then
  C_OK=$'\033[32m'; C_NG=$'\033[31m'; C_WARN=$'\033[33m'
  C_SKIP=$'\033[90m'; C_SEC=$'\033[1;36m'; C_OFF=$'\033[0m'
else
  C_OK=""; C_NG=""; C_WARN=""; C_SKIP=""; C_SEC=""; C_OFF=""
fi

NAME_WIDTH=48

# 端末上の表示幅を数える (日本語などの全角文字は 2 桁として扱う)
disp_width() {
  local s="$1" w=0 c cp i
  for (( i = 0; i < ${#s}; i++ )); do
    c="${s:i:1}"
    printf -v cp '%d' "'${c}" 2>/dev/null || cp=63
    if (( cp >= 0x1100 && ( \
          cp <= 0x115F || \
          (cp >= 0x2E80 && cp <= 0xA4CF) || \
          (cp >= 0xAC00 && cp <= 0xD7A3) || \
          (cp >= 0xF900 && cp <= 0xFAFF) || \
          (cp >= 0xFE30 && cp <= 0xFE6F) || \
          (cp >= 0xFF00 && cp <= 0xFF60) || \
          (cp >= 0xFFE0 && cp <= 0xFFE6) ) )); then
      w=$(( w + 2 ))
    else
      w=$(( w + 1 ))
    fi
  done
  printf '%d' "$w"
}

section() {
  printf "\n%s===== %s =====%s\n" "$C_SEC" "$1" "$C_OFF"
}

# report <OK|NG|WARN|SKIP> <項目名> [メッセージ]
#   OK   : 計測値などをその行に併記する
#   それ以外: 「〇〇に失敗しました。××を確認してください。」を次行にインデントして出す
report() {
  local status="$1" name="$2" msg="${3:-}"
  local color label pad len

  case "$status" in
    OK)   label="OK  "; color="$C_OK";   OK_COUNT=$((OK_COUNT + 1)) ;;
    NG)   label="NG  "; color="$C_NG";   NG_COUNT=$((NG_COUNT + 1)) ;;
    WARN) label="WARN"; color="$C_WARN"; WARN_COUNT=$((WARN_COUNT + 1)) ;;
    SKIP) label="SKIP"; color="$C_SKIP"; SKIP_COUNT=$((SKIP_COUNT + 1)) ;;
    *)    label="????"; color="" ;;
  esac

  len=$(disp_width "$name")
  if (( len < NAME_WIDTH )); then
    pad=$(printf '%*s' $((NAME_WIDTH - len)) '')
    pad=${pad// /.}
  else
    pad=" "
  fi

  if [[ "$status" == "OK" ]]; then
    printf "  %s %s %s%s%s  %s\n" "$name" "$pad" "$color" "$label" "$C_OFF" "$msg"
  else
    printf "  %s %s %s%s%s\n" "$name" "$pad" "$color" "$label" "$C_OFF"
    [[ -n "$msg" ]] && printf "       %s→ %s%s\n" "$color" "$msg" "$C_OFF"
  fi
}

info() {
  printf "       %s\n" "$1"
}

fatal() {
  printf "\n  %s%s%s\n\n" "$C_NG" "$1" "$C_OFF"
  exit 1
}

# ---------------------------------------------------------------------------
# 0. 実行環境
# ---------------------------------------------------------------------------
section "実行環境"

if ! command -v ros2 >/dev/null 2>&1; then
  report NG "ros2 コマンド" "ros2 コマンドの検出に失敗しました。/opt/ros/humble/setup.bash と install/setup.bash を source したか確認してください。"
  fatal "ROS 2 環境が読み込まれていないため中断します。"
fi
report OK "ros2 コマンド" "$(command -v ros2)"

if [[ -z "${ROS_DISTRO:-}" ]]; then
  report NG "ROS_DISTRO" "ROS_DISTRO の取得に失敗しました。/opt/ros/humble/setup.bash を source したか確認してください。"
else
  report OK "ROS_DISTRO" "$ROS_DISTRO"
fi

report OK "ROS_DOMAIN_ID" "${ROS_DOMAIN_ID:-0 (未設定)} ※Jetson と DellPC で一致させること"
report OK "RMW_IMPLEMENTATION" "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp (既定)} ※2 台で一致させること"
if [[ "$ROLE" == "dell" ]]; then
  report OK "実行ロール" "dell (足回り制御機)"
else
  report OK "実行ロール" "jetson (高レイヤ制御機)"
fi

# main_params.yaml を探す (install 側を優先。--symlink-install なら src と同一実体)
#   2 台分離後に実行機パッケージが main_executor 以外になっても拾えるよう、
#   install / src 配下の総当たりまで含めて探す。マッチしないグロブは -f で弾かれる
PARAMS_YAML=""
for candidate in \
  "${SCRIPT_DIR}/../install/main_executor/share/main_executor/config/main_params.yaml" \
  "${SCRIPT_DIR}/main_executor/config/main_params.yaml" \
  "${SCRIPT_DIR}"/../install/*/share/*/config/main_params.yaml \
  "${SCRIPT_DIR}"/*/config/main_params.yaml
do
  if [[ -f "$candidate" ]]; then
    PARAMS_YAML="$(realpath "$candidate" 2>/dev/null || echo "$candidate")"
    break
  fi
done

if [[ -z "$PARAMS_YAML" ]]; then
  report NG "main_params.yaml" "main_params.yaml の読み込みに失敗しました。src/main_executor/config/main_params.yaml が存在するか確認してください。2 台分離後に DellPC 側から main_executor を外す場合は、DellPC 用のパラメータファイルを用意してこのスクリプトの探索先に置いてください。"
  fatal "パラメータファイルが見つからないため中断します。"
fi
report OK "main_params.yaml" "$PARAMS_YAML"

# launch セクションと CAN インターフェース名を取り出す
PARAM_DUMP="$(python3 - "$PARAMS_YAML" <<'PY'
import sys
import yaml

with open(sys.argv[1]) as f:
    doc = yaml.safe_load(f) or {}

launch = (doc.get('launch') or {}).get('ros__parameters', {}) or {}
can = (doc.get('socketcan_interface_node') or {}).get('ros__parameters', {}) or {}

print('SIM=%s' % ('true' if launch.get('sim', False) else 'false'))
print('JOY=%s' % ('true' if launch.get('joy', False) else 'false'))
print('CAN_IF=%s' % (can.get('if_name', 'can0')))
PY
)"

if [[ $? -ne 0 || -z "$PARAM_DUMP" ]]; then
  report NG "main_params.yaml の解析" "main_params.yaml の解析に失敗しました。YAML の書式と python3-yaml の導入を確認してください。"
  fatal "パラメータを解釈できないため中断します。"
fi

SIM="false"; JOY="false"; CAN_IF="can0"
while IFS='=' read -r key value; do
  case "$key" in
    SIM)    SIM="$value" ;;
    JOY)    JOY="$value" ;;
    CAN_IF) CAN_IF="$value" ;;
  esac
done <<< "$PARAM_DUMP"

if [[ "$SIM" == "true" ]]; then
  report NG "launch.sim" "main_params.yaml の sim が true です。サポートしていません。"
  info "このスクリプトは実機構成 (sim: false) 専用です。"
  info "${PARAMS_YAML} の launch: セクションで sim: false に変更してから実行してください。"
  printf "\n"
  exit 1
fi
report OK "launch.sim" "false (実機構成)"
report OK "launch.joy" "$JOY"
report OK "CAN インターフェース名" "$CAN_IF"

# ---------------------------------------------------------------------------
# 1〜4. ノード / トピック / 周波数 / TF (rclpy でまとめて計測)
# ---------------------------------------------------------------------------
run_ros_checks() {
  python3 - "$MEASURE_DURATION" "$ROLE" "$JOY" <<'PY' 2>/dev/null
import sys
import time

DURATION = float(sys.argv[1])
ROLE = sys.argv[2]
JOY_ENABLED = sys.argv[3] == 'true'

DISCOVERY_WAIT = 2.0

try:
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rosidl_runtime_py.utilities import get_message
    from tf2_ros import Buffer, TransformListener
except Exception as exc:  # noqa: BLE001
    print('RES\tNG\trclpy のインポート\trclpy の読み込みに失敗しました (%s)。install/setup.bash を source したか確認してください。' % exc)
    sys.exit(1)


def emit(kind, *fields):
    print('\t'.join([kind] + [str(f) for f in fields]), flush=True)


SKIP_REASON = ('Jetson 側の担当です。Jetson 上のスクリプト (ROLE="jetson") で確認してください。'
               if ROLE == 'dell'
               else 'DellPC 側の担当です。DellPC 上のスクリプトで ROLE="dell" にして確認してください。')


def want(layer):
    if ROLE == 'dell':
        return layer in ('low', 'both')
    return layer in ('high', 'both')


# (フルノード名, レイヤ, 深刻度, ヒント)
NODES = [
    ('/controller_node',           'high', 'error', 'main_exec が起動しているか、colcon build --packages-up-to main_executor が通っているか確認してください。'),
    ('/vectormap_server_node',     'high', 'error', 'main_exec のログに map_path の読み込みエラーが出ていないか確認してください。'),
    ('/lane_line_publisher_node',  'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/vectormap_visualizer_node', 'high', 'warn',  'main_exec が起動しているか確認してください。'),
    ('/pose_estimater_node',       'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/ekf_localizer_node',        'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/odom_tf_node',              'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/map_odom_tf_node',          'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/mission_planner_node',      'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/local_planner_server_node', 'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/controller_server_node',    'high', 'error', 'main_exec が起動しているか確認してください。'),
    ('/zed_wrapper_node',          'high', 'error', 'ZED SDK が導入されているか (未検出だと zed_wrapper はビルドをスキップします)、ZED カメラが USB3 で接続されているか確認してください。'),
    ('/road_detector_node',        'high', 'error', 'road_detector (rclpy) が起動しているか、colcon build --packages-select road_detector が通っているか確認してください。'),
    ('/vectornav',                 'high', 'error', 'vectornav が起動しているか、VN-100 が /dev/ttyUSB0 に見えて読み書き権限があるか確認してください。'),
    ('/vn_sensor_msgs',            'high', 'error', 'vectornav の launch が読み込まれているか確認してください。'),
    ('/robot_state_publisher',     'high', 'error', 'robot_state_publisher が起動しているか、URDF (simulator/models/ai_car1/model.urdf) が読めているか確認してください。'),
    ('/chassis_driver_node',       'low',  'error', 'DellPC 側で chassis_driver を含む実行機が起動しているか確認してください。'),
    ('/socketcan_interface_node',  'low',  'error', 'DellPC 側で socketcan_interface_node が起動しているか、CAN インターフェースの bind に失敗していないか起動ログを確認してください。'),
    ('/odrive_axis0/can_node',     'low',  'error', 'DellPC 側で odrive_can_launch.yaml が読み込まれているか確認してください。'),
]

if JOY_ENABLED:
    NODES.insert(0, ('/joy_node', 'high', 'error',
                     'joy パッケージが導入されているか、コントローラが /dev/input/js0 に接続されているか確認してください。'))

# (トピック, 型, 種別, 期待Hz, 下限Hz, 上限Hz, レイヤ, 深刻度, ヒント)
#   種別 rate   : 周波数まで判定する
#   種別 event  : 到着したことだけを判定する (発生契機が不定のもの)
#   種別 latched: transient_local。1 サンプル受け取れれば良い
TOPICS = [
    # --- センサ ---
    ('/vectornav/imu', 'sensor_msgs/msg/Imu', 'rate', 20.0, 14.0, None, 'high', 'error',
     'IMU データの受信に失敗しました。vectornav が起動しているか、vectornav.yaml の AsyncDataOutputFrequency (20 Hz) と /dev/ttyUSB0 の接続を確認してください。'),
    ('/vectornav/gnss', 'sensor_msgs/msg/NavSatFix', 'rate', 20.0, 14.0, None, 'high', 'error',
     'GNSS データの受信に失敗しました。vectornav が起動しているか、GNSS アンテナが接続され屋外で測位できているか確認してください。'),
    # 2 台構成では Jetson が出版し DellPC の chassis_driver も購読する
    ('/vectornav/velocity_body', 'geometry_msgs/msg/TwistWithCovarianceStamped', 'rate', 20.0, 14.0, None, 'both', 'error',
     '車体速度の受信に失敗しました。vectornav の INS グループ出力が有効か、vn_sensor_msgs が起動しているか確認してください。2 台構成では Jetson 側で出版されるため、DellPC で見えない場合は 2 台間の DDS 疎通を確認してください。'),
    ('/joy', 'sensor_msgs/msg/Joy', 'event', None, None, None, 'high', 'error',
     'コントローラ入力の受信に失敗しました。joy_node が起動しているか、コントローラが /dev/input/js0 として認識されているか確認してください。'),

    # --- カメラ ---
    ('/zed/zed_node/rgb/image_rect_color', 'sensor_msgs/msg/Image', 'rate', 15.0, 10.0, None, 'high', 'error',
     'カメラ画像の受信に失敗しました。ZED SDK が導入され zed_wrapper が ENABLE_ZED 付きでビルドされているか、カメラが USB3 ポートに接続されているか確認してください。'),
    ('/zed/zed_node/point_cloud', 'sensor_msgs/msg/PointCloud2', 'rate', 15.0, 10.0, None, 'high', 'warn',
     '点群の受信に失敗しました。zed_wrapper_node の depth mode 設定と GPU メモリの空きを確認してください。'),

    # --- 知覚 ---
    ('/perception/lane_mask', 'sensor_msgs/msg/Image', 'rate', 15.0, 8.0, None, 'high', 'error',
     '白線マスクの受信に失敗しました。road_detector_node が起動しているか、入力 /zed/zed_node/rgb/image_rect_color が出ているか、GPU 推論でエラーが出ていないか確認してください。'),
    ('/perception/lane_mask_visualize', 'sensor_msgs/msg/Image', 'rate', 15.0, 8.0, None, 'high', 'warn',
     '白線マスク可視化の受信に失敗しました。road_detector_node のログを確認してください。'),
    ('/perception/lane_line_points', 'sensor_msgs/msg/PointCloud2', 'rate', 15.0, 8.0, None, 'high', 'error',
     '白線点群の受信に失敗しました。lane_line_publisher_node が /perception/lane_mask を受け取れているか、mask_threshold (既定 128) が高すぎないか確認してください。'),
    ('/perception/lane_line', 'visualization_msgs/msg/MarkerArray', 'rate', 15.0, 8.0, None, 'high', 'warn',
     '白線マーカの受信に失敗しました。lane_line_publisher_node のログを確認してください。'),
    ('/perception/vectormap_visualize', 'sensor_msgs/msg/Image', 'rate', 15.0, 8.0, None, 'high', 'warn',
     'ベクターマップ重畳画像の受信に失敗しました。vectormap_visualizer_node が TF base_link->map を引けているか確認してください。'),
    ('/perception/objects', 'object_detection_msgs/msg/ObjectInfoArray', 'event', None, None, None, 'high', 'warn',
     '障害物情報の受信に失敗しました。object_detector_node は main_executor/src/main.cpp でコメントアウトされています。障害物回避を使う場合は有効化してください。'),

    # --- 地図 ---
    ('/vector_map', 'vectormap_msgs/msg/VectorMap', 'latched', None, None, None, 'high', 'error',
     'ベクターマップの受信に失敗しました。vectormap_server_node が起動しているか、main_params.yaml の map_path (aiformula_course.osm) が map パッケージに存在するか確認してください。'),
    ('/vector_map/visualize', 'visualization_msgs/msg/MarkerArray', 'latched', None, None, None, 'high', 'warn',
     'ベクターマップ可視化の受信に失敗しました。vectormap_server_node のログを確認してください。'),

    # --- 自己位置 ---
    ('/localization/pf_pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', 'rate', 50.0, 35.0, None, 'high', 'error',
     'パーティクルフィルタ推定値の受信に失敗しました。pose_estimater_node が /vector_map と /perception/lane_line_points を受け取れているか確認してください。'),
    ('/localization/particle', 'geometry_msgs/msg/PoseArray', 'rate', 50.0, 35.0, None, 'high', 'warn',
     'パーティクル分布の受信に失敗しました。pose_estimater_node のログを確認してください。'),
    ('/localization/pose', 'geometry_msgs/msg/PoseWithCovarianceStamped', 'rate', 50.0, 35.0, None, 'high', 'error',
     'EKF 推定姿勢の受信に失敗しました。ekf_localizer_node が /localization/pf_pose と /vectornav/velocity_body を受け取れているか確認してください。'),
    ('/localization/odom', 'nav_msgs/msg/Odometry', 'rate', 50.0, 35.0, None, 'high', 'error',
     'オドメトリの受信に失敗しました。odom_tf_node が /vectornav/imu と /vectornav/velocity_body を受け取れているか確認してください。'),

    # --- 計画 ---
    ('/planner/global_path', 'nav_msgs/msg/Path', 'rate', 10.0, 7.0, None, 'high', 'error',
     'グローバル経路の受信に失敗しました。mission_planner_node が /vector_map と /localization/pose を受け取れているか、自車が経路上のレーンレット近傍にいるか確認してください。'),
    ('/planner/local_path', 'nav_msgs/msg/Path', 'rate', 10.0, 7.0, None, 'high', 'error',
     'ローカル経路の受信に失敗しました。local_planner_server_node が /planner/global_path を受け取れているか、local_planner_plugin の名前が正しいか確認してください。'),

    # --- 制御 ---
    ('/cmd_vel', 'steered_drive_msg/msg/SteeredDrive', 'rate', 20.0, 14.0, None, 'both', 'error',
     '速度指令の受信に失敗しました。controller_server_node が起動しているか、controller_plugin の名前が正しいか確認してください。'),
    ('/autonomous', 'std_msgs/msg/Bool', 'event', None, None, None, 'high', 'warn',
     '自律走行フラグの受信に失敗しました。コントローラの Share ボタンで自律走行に切り替えたか確認してください。'),
    ('/planning/nav_cmd', 'std_msgs/msg/String', 'event', None, None, None, 'high', 'warn',
     '進路指令の受信に失敗しました。controller_node がコントローラ入力を受け取れているか確認してください。'),
    # 2 台構成では Jetson の controller_node が出版し DellPC の chassis_driver が購読する
    ('/restart', 'std_msgs/msg/Empty', 'event', None, None, None, 'both', 'warn',
     '再起動指令の受信に失敗しました。コントローラで再起動を送るまでは出版されないため、走行前であれば正常です。DellPC 側で見えない場合は 2 台間の DDS 疎通を確認してください。'),
    # 2 台構成では DellPC が出版し Jetson の controller_server が購読する
    ('/caster_data', 'std_msgs/msg/Float64MultiArray', 'rate', 500.0, 250.0, None, 'both', 'warn',
     'キャスタ状態の受信に失敗しました。chassis_driver_node は停止モード中は出版しません。コントローラで再起動 (restart) を送り、/cmd_vel が届いているか確認してください。Jetson 側で見えない場合は 2 台間の DDS 疎通も確認してください。'),
    ('/caster_odom', 'nav_msgs/msg/Odometry', 'rate', 500.0, 250.0, None, 'low', 'warn',
     'キャスタオドメトリの受信に失敗しました。chassis_driver_node はキャスタ回転エンコーダ (/can_rx_013) を 1 回も受信していないと出版しません。CAN 配線を確認してください。なお現状このトピックの購読者はいません。'),

    # --- CAN トピック ---
    ('/can_tx', 'socketcan_interface_msg/msg/SocketcanIF', 'rate', 500.0, 250.0, None, 'low', 'error',
     'CAN 送信フレームの受信に失敗しました。chassis_driver_node が起動しているか (interval_ms: 2 = 500 Hz)、main_exec のログを確認してください。'),
    ('/can_rx_012', 'socketcan_interface_msg/msg/SocketcanIF', 'event', None, None, None, 'low', 'error',
     'キャスタ操舵角エンコーダ (CAN ID 0x012) の受信に失敗しました。socketcan_interface_node は受信した ID のトピックしか作りません。エンコーダ基板の電源と CAN 配線、終端抵抗を確認してください。'),
    ('/can_rx_013', 'socketcan_interface_msg/msg/SocketcanIF', 'event', None, None, None, 'low', 'error',
     'キャスタ回転エンコーダ (CAN ID 0x013) の受信に失敗しました。エンコーダ基板の電源と CAN 配線、終端抵抗を確認してください。'),
    ('/can_rx_712', 'socketcan_interface_msg/msg/SocketcanIF', 'event', None, None, None, 'low', 'warn',
     '非常停止基板 (CAN ID 0x712) の受信に失敗しました。非常停止基板の電源と CAN 配線を確認してください。'),
    ('/can_rx_301', 'socketcan_interface_msg/msg/SocketcanIF', 'event', None, None, None, 'low', 'error',
     'ODrive ハートビート (CAN ID 0x301 = node_id 24) の受信に失敗しました。ODrive の電源、node_id 設定 (24)、CAN ボーレートが一致しているか確認してください。'),
    ('/can_rx_309', 'socketcan_interface_msg/msg/SocketcanIF', 'event', None, None, None, 'low', 'warn',
     'ODrive エンコーダ推定値 (CAN ID 0x309) の受信に失敗しました。ODrive の周期送信設定 (encoder_estimates rate) を確認してください。'),
    ('/odrive_axis0/control_message', 'odrive_can/msg/ControlMessage', 'rate', 500.0, 250.0, None, 'low', 'warn',
     'ODrive 制御指令の受信に失敗しました。chassis_driver_node は停止モード中は出版しません。コントローラで再起動 (restart) を送ったか確認してください。'),
    ('/odrive_axis0/controller_status', 'odrive_can/msg/ControllerStatus', 'event', None, None, None, 'low', 'warn',
     'ODrive コントローラ状態の受信に失敗しました。/can_rx_301 と /can_rx_309 が来ているか確認してください。'),
    ('/odrive_axis0/odrive_status', 'odrive_can/msg/ODriveStatus', 'event', None, None, None, 'low', 'warn',
     'ODrive 本体状態の受信に失敗しました。/can_rx_303 (エラー) と /can_rx_317 (バス電圧) が来ているか確認してください。'),
]

# (親, 子, 静的か, 許容遅延[s], ヒント)
TF_PAIRS = [
    ('map', 'odom', False, 0.5,
     'map->odom の取得に失敗しました。map_odom_tf_node が /localization/pose を受け取れているか、odom->base_link が先に出ているか確認してください。'),
    ('odom', 'base_link', False, 0.2,
     'odom->base_link の取得に失敗しました。odom_tf_node が /vectornav/imu と /vectornav/velocity_body を受け取れているか確認してください。'),
    ('map', 'base_link', False, 0.5,
     'map->base_link の取得に失敗しました。map->odom と odom->base_link のどちらかが欠けています。上 2 項目の結果を確認してください。'),
    ('base_link', 'camera_link', True, None,
     'base_link->camera_link (静的 TF) の取得に失敗しました。robot_state_publisher が起動しているか、URDF に camera_joint があるか確認してください。'),
    ('base_link', 'imu_link', True, None,
     'base_link->imu_link (静的 TF) の取得に失敗しました。robot_state_publisher が起動しているか確認してください。'),
    ('base_link', 'gps_link', True, None,
     'base_link->gps_link (静的 TF) の取得に失敗しました。robot_state_publisher が起動しているか確認してください。'),
]

REL_NAME = {ReliabilityPolicy.RELIABLE: 'reliable', ReliabilityPolicy.BEST_EFFORT: 'best_effort'}
DUR_NAME = {DurabilityPolicy.VOLATILE: 'volatile', DurabilityPolicy.TRANSIENT_LOCAL: 'transient_local'}

rclpy.init(args=None)
node = rclpy.create_node('aiformula_system_check')
executor = SingleThreadedExecutor()
executor.add_node(node)

tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
tf_listener = TransformListener(tf_buffer, node, spin_thread=False)


def spin_for(seconds):
    end = time.monotonic() + seconds
    while rclpy.ok() and time.monotonic() < end:
        executor.spin_once(timeout_sec=0.05)


# --- ディスカバリ待ち ---
spin_for(DISCOVERY_WAIT)

# =========================== 1. ノード =====================================
emit('SEC', 'ノード')

live_nodes = set()
for name, namespace in node.get_node_names_and_namespaces():
    ns = namespace if namespace.endswith('/') else namespace + '/'
    live_nodes.add(ns + name)

for full_name, layer, severity, hint in NODES:
    if not want(layer):
        emit('RES', 'SKIP', full_name, SKIP_REASON)
        continue
    if full_name in live_nodes:
        emit('RES', 'OK', full_name, '起動中')
    else:
        emit('RES', 'NG' if severity == 'error' else 'WARN', full_name,
             '%s の検出に失敗しました。%s' % (full_name, hint))

# =========================== 2. トピック ===================================
emit('SEC', 'トピック / 周波数 (計測 %.1f 秒)' % DURATION)

graph_types = dict(node.get_topic_names_and_types())

targets = []   # (spec, 状態)
subs = []
stats = {}     # topic -> [count, first_t, last_t]

for spec in TOPICS:
    topic, expected_type, kind, expect_hz, min_hz, max_hz, layer, severity, hint = spec

    if not want(layer):
        targets.append((spec, 'skip', SKIP_REASON))
        continue

    if topic not in graph_types:
        targets.append((spec, 'absent', None))
        continue

    actual_types = graph_types[topic]
    if expected_type not in actual_types:
        targets.append((spec, 'type', '/'.join(actual_types)))
        continue

    pub_infos = node.get_publishers_info_by_topic(topic)
    if not pub_infos:
        targets.append((spec, 'nopub', None))
        continue

    durability = DurabilityPolicy.VOLATILE
    if all(i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in pub_infos):
        durability = DurabilityPolicy.TRANSIENT_LOCAL

    qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=50,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=durability,
    )

    stats[topic] = [0, None, None]

    def make_cb(key):
        def cb(_msg):
            s = stats[key]
            now = time.monotonic()
            s[0] += 1
            if s[1] is None:
                s[1] = now
            s[2] = now
        return cb

    try:
        msg_cls = get_message(expected_type)
    except Exception as exc:  # noqa: BLE001
        targets.append((spec, 'notype', str(exc)))
        continue

    subs.append(node.create_subscription(msg_cls, topic, make_cb(topic), qos, raw=True))

    # QoS 不整合 (BEST_EFFORT 出版 x RELIABLE 購読 / VOLATILE 出版 x TRANSIENT_LOCAL 購読)
    mismatch = []
    for s in node.get_subscriptions_info_by_topic(topic):
        if s.node_name == node.get_name():
            continue
        for p in pub_infos:
            if (p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
                    and s.qos_profile.reliability == ReliabilityPolicy.RELIABLE):
                mismatch.append('%s の reliability が reliable (出版側は best_effort)' % s.node_name)
            if (p.qos_profile.durability == DurabilityPolicy.VOLATILE
                    and s.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL):
                mismatch.append('%s の durability が transient_local (出版側は volatile)' % s.node_name)

    targets.append((spec, 'measuring', sorted(set(mismatch))))

# --- 計測 ---
spin_for(DURATION)

for spec, state, extra in targets:
    topic, expected_type, kind, expect_hz, min_hz, max_hz, layer, severity, hint = spec
    ng = 'NG' if severity == 'error' else 'WARN'

    if state == 'skip':
        emit('RES', 'SKIP', topic, extra)
        continue
    if state == 'absent':
        emit('RES', ng, topic, 'トピック %s が存在しません。%s' % (topic, hint))
        continue
    if state == 'type':
        emit('RES', ng, topic, '型が期待と異なります (期待 %s / 実際 %s)。出版ノードのメッセージ定義を確認してください。' % (expected_type, extra))
        continue
    if state == 'nopub':
        emit('RES', ng, topic, '%s の出版者が 0 です。出版ノードが起動しているか確認してください。' % topic)
        continue
    if state == 'notype':
        emit('RES', ng, topic, 'メッセージ型 %s の読み込みに失敗しました (%s)。該当 msg パッケージがビルド済みか確認してください。' % (expected_type, extra))
        continue

    count, first_t, last_t = stats[topic]
    hz = 0.0
    if count >= 2 and last_t is not None and first_t is not None and last_t > first_t:
        hz = (count - 1) / (last_t - first_t)

    pub_n = node.count_publishers(topic)

    if count == 0:
        emit('RES', ng, topic,
             '%s のデータ受信に失敗しました (出版者 %d)。%s' % (topic, pub_n, hint))
        continue

    if kind == 'latched':
        emit('RES', 'OK', topic, 'ラッチ受信 %d 件 (transient_local, pub=%d)' % (count, pub_n))
    elif kind == 'event':
        emit('RES', 'OK', topic, '%d 件受信 / %.1f Hz (pub=%d)' % (count, hz, pub_n))
    else:
        lo = min_hz if min_hz is not None else expect_hz * 0.7
        hi = max_hz
        if hz < lo:
            emit('RES', ng, topic,
                 '周波数が不足しています (実測 %.1f Hz / 期待 %.1f Hz / 下限 %.1f Hz)。%s' % (hz, expect_hz, lo, hint))
        elif hi is not None and hz > hi:
            emit('RES', 'WARN', topic,
                 '周波数が過大です (実測 %.1f Hz / 期待 %.1f Hz / 上限 %.1f Hz)。出版者が二重に起動していないか確認してください。' % (hz, expect_hz, hi))
        else:
            emit('RES', 'OK', topic, '%.1f Hz (期待 %.1f Hz, pub=%d)' % (hz, expect_hz, pub_n))

    if extra:
        for m in extra:
            emit('RES', 'WARN', topic + ' (QoS)',
                 'QoS が不整合です: %s。出版側と購読側の QoS 設定を合わせてください。' % m)

# =========================== 3. TF =========================================
emit('SEC', 'TF')

if ROLE == 'dell':
    for parent, child, is_static, max_delay, hint in TF_PAIRS:
        emit('RES', 'SKIP', '%s -> %s' % (parent, child), SKIP_REASON)
else:
    now = node.get_clock().now()
    for parent, child, is_static, max_delay, hint in TF_PAIRS:
        label = '%s -> %s' % (parent, child)
        try:
            tf = tf_buffer.lookup_transform(parent, child, Time())
        except Exception as exc:  # noqa: BLE001
            emit('RES', 'NG', label, '%s (%s)' % (hint, type(exc).__name__))
            continue

        t = tf.transform.translation
        pose = 'x=%.3f y=%.3f z=%.3f' % (t.x, t.y, t.z)

        if is_static or max_delay is None:
            emit('RES', 'OK', label, '%s (静的 TF)' % pose)
            continue

        stamp = Time.from_msg(tf.header.stamp)
        if stamp.nanoseconds == 0:
            # /tf_static で配信されていると lookup 時刻が 0 のまま返る
            emit('RES', 'WARN', label,
                 '%s はタイムスタンプが 0 です。動的 TF のはずが /tf_static で配信されていないか確認してください。' % label)
            continue

        delay = (now - stamp).nanoseconds / 1e9
        if delay > max_delay:
            emit('RES', 'NG', label,
                 'TF が古くなっています (遅延 %.2f 秒 / 許容 %.2f 秒)。%s' % (delay, max_delay, hint))
        else:
            emit('RES', 'OK', label, '%s (遅延 %.3f 秒)' % (pose, delay))

executor.shutdown()
node.destroy_node()
rclpy.shutdown()
PY
}

dispatch_ros_checks() {
  local kind status name msg
  local produced=0
  while IFS=$'\t' read -r kind status name msg; do
    produced=1
    case "$kind" in
      SEC)  section "$status" ;;
      RES)  report "$status" "$name" "$msg" ;;
      INFO) info "$status" ;;
    esac
  done < <(run_ros_checks)

  if [[ "$produced" -eq 0 ]]; then
    section "ノード / トピック / TF"
    report NG "rclpy チェック" "ROS 2 の状態取得に失敗しました。install/setup.bash を source したか、python3-yaml と tf2_ros が導入されているか確認してください。"
  fi
}

dispatch_ros_checks

# ---------------------------------------------------------------------------
# 5. CAN
# ---------------------------------------------------------------------------
section "CAN (${CAN_IF})"

check_can() {
  if [[ "$ROLE" == "jetson" ]]; then
    report SKIP "CAN バス" "DellPC 側の担当です。DellPC 上のスクリプトで ROLE=\"dell\" にして確認してください。"
    return
  fi

  if ! command -v ip >/dev/null 2>&1; then
    report NG "ip コマンド" "ip コマンドの検出に失敗しました。iproute2 が導入されているか確認してください。"
    return
  fi

  local link
  if ! link="$(ip -details link show "$CAN_IF" 2>/dev/null)"; then
    report NG "インターフェース ${CAN_IF}" "CAN インターフェース ${CAN_IF} の検出に失敗しました。USB-CAN アダプタが接続されているか、\`sudo ip link set ${CAN_IF} up type can bitrate 1000000\` を実行したか確認してください。"
    info "udev で名前を固定している場合は /etc/udev/rules.d の設定も確認してください。"
    return
  fi
  report OK "インターフェース ${CAN_IF}" "存在"

  if grep -qE '\bstate UP\b' <<< "$link"; then
    report OK "リンク状態" "UP"
  else
    report NG "リンク状態" "CAN インターフェース ${CAN_IF} が UP になっていません。\`sudo ip link set ${CAN_IF} up type can bitrate 1000000\` を実行したか確認してください。"
    return
  fi

  local bitrate
  bitrate="$(grep -oP '\bbitrate \K[0-9]+' <<< "$link" | head -n1)"
  if [[ -n "$bitrate" ]]; then
    report OK "ビットレート" "${bitrate} bps ※基板側の設定と一致していること"
  else
    report WARN "ビットレート" "ビットレートの取得に失敗しました。仮想 CAN (vcan) を使っていないか、bitrate を指定して up したか確認してください。"
  fi

  local can_state
  can_state="$(grep -oP 'can state \K[A-Z-]+' <<< "$link" | head -n1)"
  case "$can_state" in
    ERROR-ACTIVE)
      report OK "CAN コントローラ状態" "$can_state"
      ;;
    ERROR-WARNING|ERROR-PASSIVE)
      report NG "CAN コントローラ状態" "CAN コントローラがエラー状態 (${can_state}) です。終端抵抗 (両端 120Ω)、CAN_H / CAN_L の結線、ビットレートの一致を確認してください。"
      ;;
    BUS-OFF)
      report NG "CAN コントローラ状態" "CAN が BUS-OFF です。配線の短絡とビットレートの不一致を確認し、\`sudo ip link set ${CAN_IF} down && sudo ip link set ${CAN_IF} up type can bitrate ${bitrate:-1000000}\` で復旧してください。"
      ;;
    "")
      report WARN "CAN コントローラ状態" "CAN コントローラ状態の取得に失敗しました。仮想 CAN (vcan) の可能性があります。"
      ;;
    *)
      report WARN "CAN コントローラ状態" "想定外の状態 (${can_state}) です。dmesg にドライバのエラーが出ていないか確認してください。"
      ;;
  esac

  local berr_tx berr_rx
  berr_tx="$(grep -oP 'berr-counter tx \K[0-9]+' <<< "$link" | head -n1)"
  berr_rx="$(grep -oP 'berr-counter tx [0-9]+ rx \K[0-9]+' <<< "$link" | head -n1)"
  if [[ -n "$berr_tx" && -n "$berr_rx" ]]; then
    if (( berr_tx == 0 && berr_rx == 0 )); then
      report OK "エラーカウンタ" "tx=0 rx=0"
    else
      report NG "エラーカウンタ" "CAN エラーカウンタが増えています (tx=${berr_tx} rx=${berr_rx})。終端抵抗と結線、ノイズ対策を確認してください。"
    fi
  fi

  # RX / TX パケットの増分でバスが生きているかを見る (can-utils 不要)
  local stats_before stats_after
  local rx0 tx0 rx1 tx1 rx_delta tx_delta
  stats_before="$(ip -s link show "$CAN_IF" 2>/dev/null)"
  rx0="$(awk '/RX:/{getline; print $2}' <<< "$stats_before")"
  tx0="$(awk '/TX:/{getline; print $2}' <<< "$stats_before")"
  sleep 1
  stats_after="$(ip -s link show "$CAN_IF" 2>/dev/null)"
  rx1="$(awk '/RX:/{getline; print $2}' <<< "$stats_after")"
  tx1="$(awk '/TX:/{getline; print $2}' <<< "$stats_after")"

  if [[ -n "$rx0" && -n "$rx1" ]]; then
    rx_delta=$((rx1 - rx0))
    if (( rx_delta > 0 )); then
      report OK "CAN 受信" "${rx_delta} フレーム/秒"
    else
      report NG "CAN 受信" "CAN フレームの受信に失敗しました (1 秒間で 0 フレーム)。エンコーダ基板と ODrive の電源、CAN 配線、終端抵抗を確認してください。"
    fi
  else
    report WARN "CAN 受信" "受信統計の取得に失敗しました。ip -s link show ${CAN_IF} の出力形式を確認してください。"
  fi

  if [[ -n "$tx0" && -n "$tx1" ]]; then
    tx_delta=$((tx1 - tx0))
    if (( tx_delta > 0 )); then
      report OK "CAN 送信" "${tx_delta} フレーム/秒"
    else
      report NG "CAN 送信" "CAN フレームの送信に失敗しました (1 秒間で 0 フレーム)。chassis_driver_node と socketcan_interface_node が起動しているか、/can_tx が出版されているか確認してください。"
    fi
  else
    report WARN "CAN 送信" "送信統計の取得に失敗しました。ip -s link show ${CAN_IF} の出力形式を確認してください。"
  fi

  # candump があれば実際に流れている CAN ID を一覧する
  if ! command -v candump >/dev/null 2>&1; then
    report WARN "candump" "candump の検出に失敗しました。CAN ID ごとの確認には \`sudo apt install can-utils\` が必要です。"
    return
  fi

  local dump ids
  dump="$(timeout 3 candump -T 2000 "$CAN_IF" 2>/dev/null)"
  ids="$(awk '{print $2}' <<< "$dump" | sort -u | tr '\n' ' ')"
  if [[ -n "${ids// /}" ]]; then
    report OK "受信 CAN ID" "$ids"
    for expected_id in 012 013 301; do
      if grep -qw "$expected_id" <<< "$ids"; then
        report OK "CAN ID 0x${expected_id}" "受信中"
      else
        case "$expected_id" in
          012) report NG "CAN ID 0x012" "キャスタ操舵角エンコーダ (0x012) の受信に失敗しました。エンコーダ基板の電源と CAN 配線を確認してください。" ;;
          013) report NG "CAN ID 0x013" "キャスタ回転エンコーダ (0x013) の受信に失敗しました。エンコーダ基板の電源と CAN 配線を確認してください。" ;;
          301) report NG "CAN ID 0x301" "ODrive ハートビート (0x301) の受信に失敗しました。ODrive の電源と node_id 設定 (24) を確認してください。" ;;
        esac
      fi
    done
  else
    report NG "受信 CAN ID" "candump で 2 秒間 1 フレームも受信できませんでした。バス上の機器の電源と終端抵抗を確認してください。"
  fi
}

check_can

# ---------------------------------------------------------------------------
# 6. DellPC (足回り制御機) との接続
# ---------------------------------------------------------------------------
section "${PEER_NAME} 接続 (${PEER_IP})"

check_network() {
  local subnet="${PEER_IP%.*}."
  local local_addrs
  local_addrs="$(ip -4 -brief addr show 2>/dev/null | awk '{for(i=3;i<=NF;i++) print $1" "$i}')"

  if grep -qF " ${OWN_IP}/" <<< "$local_addrs"; then
    report OK "自機の IP" "${OWN_IP} (${OWN_NAME})"
  else
    local own_addr
    own_addr="$(grep -F " ${subnet}" <<< "$local_addrs" | head -n1)"
    report NG "自機の IP" "自機に ${OWN_IP} が設定されていません (現在: ${own_addr:-${subnet}0/24 に該当なし})。${OWN_NAME} の固定 IP が ${OWN_IP}/24 になっているか、ROLE の設定が実機と合っているか確認してください。"
  fi

  if ping -c 3 -W 1 "$PEER_IP" >/dev/null 2>&1; then
    local rtt
    rtt="$(ping -c 3 -W 1 "$PEER_IP" 2>/dev/null | tail -n1 | awk -F'/' '{print $5}')"
    report OK "ping ${PEER_IP}" "${PEER_NAME} まで 平均 RTT ${rtt:-?} ms"
  else
    report NG "ping ${PEER_IP}" "${PEER_NAME} (${PEER_IP}) への疎通に失敗しました。LAN ケーブルの接続、${PEER_NAME} の電源、固定 IP 設定 (${PEER_IP}/24) を確認してください。"
    info "自機と ${PEER_NAME} が同一サブネット (${subnet}0/24) にいるかも確認してください。"
    return
  fi

  local neigh
  neigh="$(ip neigh show "$PEER_IP" 2>/dev/null | head -n1)"
  if [[ -n "$neigh" ]]; then
    report OK "ARP エントリ" "$neigh"
  else
    report WARN "ARP エントリ" "ARP エントリの取得に失敗しました。L2 の到達性が不安定な可能性があります。"
  fi

  if timeout 2 bash -c "echo > /dev/tcp/${PEER_IP}/22" 2>/dev/null; then
    report OK "SSH (22/tcp)" "接続可"
  else
    report WARN "SSH (22/tcp)" "${PEER_NAME} の SSH ポートへの接続に失敗しました。遠隔でログを見る場合は sshd の起動とファイアウォールを確認してください。"
  fi

  # マルチキャストが通らないと DDS のディスカバリが成立しない
  if ping -c 2 -W 1 224.0.0.1 2>/dev/null | grep -q "$PEER_IP"; then
    report OK "マルチキャスト" "${PEER_NAME} から応答あり (DDS ディスカバリ可)"
  else
    report WARN "マルチキャスト" "マルチキャスト (224.0.0.1) への ${PEER_NAME} からの応答確認に失敗しました。ROS 2 のディスカバリはマルチキャストを使うため、スイッチの IGMP スヌーピングとファイアウォールを確認してください。"
  fi

  report OK "ROS_DOMAIN_ID の整合" "自機は ${ROS_DOMAIN_ID:-0}。${PEER_NAME} 側でも同じ値になっているか確認してください。"
}

check_network

# ---------------------------------------------------------------------------
# サマリ
# ---------------------------------------------------------------------------
TOTAL=$((OK_COUNT + NG_COUNT + WARN_COUNT + SKIP_COUNT))

printf "\n%s===== 結果 =====%s\n" "$C_SEC" "$C_OFF"
printf "  合計 %d 項目 : %sOK %d%s / %sNG %d%s / %sWARN %d%s / %sSKIP %d%s\n\n" \
  "$TOTAL" \
  "$C_OK" "$OK_COUNT" "$C_OFF" \
  "$C_NG" "$NG_COUNT" "$C_OFF" \
  "$C_WARN" "$WARN_COUNT" "$C_OFF" \
  "$C_SKIP" "$SKIP_COUNT" "$C_OFF"

if (( NG_COUNT > 0 )); then
  printf "  %sNG が %d 件あります。上の → の指示を確認してください。%s\n\n" "$C_NG" "$NG_COUNT" "$C_OFF"
  exit 1
fi

if (( WARN_COUNT > 0 )); then
  printf "  %s致命的な NG はありません。WARN %d 件は運用状況によっては正常です。%s\n\n" "$C_WARN" "$WARN_COUNT" "$C_OFF"
fi

exit 0
