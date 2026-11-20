import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def resolve_python() -> str:
    """ROS の python (/usr/bin/python3) には torch が入っていないため、
    torch のある python があればそれを prefix にして起動する。
    見つからなければ空文字を返し、従来どおり shebang の python に任せる。"""
    candidates = [
        os.environ.get('E2E_PLANNER_PYTHON', ''),
        os.path.expanduser('~/miniforge3/envs/aiformula/bin/python'),
    ]
    for candidate in candidates:
        if candidate and os.path.exists(candidate):
            return candidate
    return ''


def tensorrt_pylon_detector_built() -> bool:
    """object_detection の pylon_detector (C++、TensorRT) がビルドされているか。
    TensorRT の無い PC では object_detection は空のパッケージになり、main_executor も組み込まない"""
    try:
        from ament_index_python.packages import get_package_prefix
        prefix = get_package_prefix('object_detection')
    except Exception:
        return False
    return os.path.exists(os.path.join(prefix, 'lib', 'libpylon_detector_node.so'))


def launch_setup(context, *args, **kwargs):
    sim_flag = LaunchConfiguration('sim_flag').perform(context)

    if sim_flag.lower() == 'true':
        executable_name = 'inference_node_sim.py'
    else:
        executable_name = 'inference_node'

    inference_node = Node(
        package='e2e_planner',
        executable=executable_name,
        name='inference_node',
        output='screen',
        prefix=resolve_python(),
        parameters=[{
            'model_name': LaunchConfiguration('model_name').perform(context),
            'num_mask_channels': int(LaunchConfiguration('num_mask_channels').perform(context)),
            'device': LaunchConfiguration('device').perform(context),
            'interval_ms': 100,
            'frame_stack_distances': LaunchConfiguration('frame_stack_distances').perform(context),
            'past_mask_distances': LaunchConfiguration('past_mask_distances').perform(context),
            'past_mask_mode': LaunchConfiguration('past_mask_mode').perform(context),
            'pylon_bbox_on_rgb': LaunchConfiguration('pylon_bbox_on_rgb').perform(context).lower() == 'true',
        }]
    )
    actions = [inference_node]

    # /perception/pylon_mask を出すもの。実機は main_executor の pylon_detector (TensorRT) が出すので起動しない
    pylon_onnx = LaunchConfiguration('pylon_onnx').perform(context).lower()
    if pylon_onnx == 'true' or (pylon_onnx == 'auto' and not tensorrt_pylon_detector_built()):
        actions.append(Node(
            package='e2e_planner',
            executable='pylon_mask_onnx_node',
            name='pylon_mask_onnx_node',
            output='screen',
            prefix=resolve_python(),
        ))
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'model_name',
            default_value='e2e_model_1027aug_early_1027aug_early_c.pt',
            description='weights/ 以下の重みファイル名'
        ),
        DeclareLaunchArgument(
            'num_mask_channels',
            default_value='2',
            description='重みの入力のマスク数。2 = 白線 + パイロン、1 = 白線だけ (feat/rgb_mask の重み)'
        ),
        # 過去フレーム。重みの学習設定 (config/sweep_1027/early_hdrop30_os1_lw2_aug1027.yaml) の
        # frame_stack_distances / past_mask_distances / past_mask_mode と同じにする。
        # 今のフレームだけの重み (e2e_model_pylon.pt など) では frame_stack_distances:='' past_mask_distances:=''
        DeclareLaunchArgument(
            'frame_stack_distances',
            default_value='1.5,3.0',
            description='過去の RGB を何 m 前から足すか (カンマ区切り)。空なら足さない'
        ),
        DeclareLaunchArgument(
            'past_mask_distances',
            default_value='1.5,3.0',
            description='過去のマスクを何 m 前から足すか (カンマ区切り)。空なら足さない'
        ),
        DeclareLaunchArgument(
            'past_mask_mode',
            default_value='merged',
            description='過去のマスクの描き方 (obs / merged / merged_inv / same / separate)'
        ),
        DeclareLaunchArgument(
            'pylon_bbox_on_rgb',
            default_value='true',
            description='RGB にパイロンの枠線を描くか。学習設定の pylon_bbox_on_rgb と同じにする '
                        '(1027_early 系は true、e2e_model_pylon.pt など枠線なしの重みは false)'
        ),
        DeclareLaunchArgument(
            'pylon_onnx',
            default_value='auto',
            description='YOLOX (onnx, CPU) で /perception/pylon_mask を出すか。'
                        'auto = TensorRT の pylon_detector がビルドされていないときだけ (シミュレーションの PC)'
        ),
        DeclareLaunchArgument(
            'device',
            default_value='cuda',
            description='推論に使うデバイス。GPU の無い PC で確かめるときだけ cpu'
        ),
        DeclareLaunchArgument(
            'sim_flag',
            default_value='false',
            description='Flag to use simulation inference node'
        ),
        OpaqueFunction(function=launch_setup)
    ])
