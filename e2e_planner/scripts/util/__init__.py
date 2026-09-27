from .slit_aug import crop_images, rotate_waypoints, augment
from .preprocess import (
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    to_bgr,
    extract_red_mask,
    preprocess_mask,
    preprocess_rgb,
    resize_for_debug,
    normalize_waypoints,
    denormalize_waypoints,
)

# 重い/環境依存の依存を持つモジュールは遅延 import する。
# yolop_processor は torch を、waypoints は pymap3d を要求するが、
# torch は学習用 venv にしか無く、pymap3d はロボット側にしか無い。
# ここで両方を eager に読むと、どちらの環境でも util を import しただけで落ちる。
_LAZY = {
    'YOLOPv2Processor': ('.yolop_processor', 'YOLOPv2Processor'),
    'PoseSample': ('.waypoints', 'PoseSample'),
    'build_forward_arc': ('.waypoints', 'build_forward_arc'),
    'interpolate_pose': ('.waypoints', 'interpolate_pose'),
    'pose_at_distance': ('.waypoints', 'pose_at_distance'),
    'transform_to_robot_frame': ('.waypoints', 'transform_to_robot_frame'),
}


def __getattr__(name):
    if name in _LAZY:
        from importlib import import_module
        module_name, attribute = _LAZY[name]
        return getattr(import_module(module_name, __name__), attribute)
    raise AttributeError(f'module {__name__!r} has no attribute {name!r}')


__all__ = [
    'crop_images', 'rotate_waypoints', 'augment', 'YOLOPv2Processor',
    'IMAGE_WIDTH', 'IMAGE_HEIGHT', 'to_bgr', 'extract_red_mask',
    'preprocess_mask', 'preprocess_rgb', 'resize_for_debug',
    'normalize_waypoints', 'denormalize_waypoints',
    'PoseSample', 'build_forward_arc', 'interpolate_pose',
    'pose_at_distance', 'transform_to_robot_frame',
]
