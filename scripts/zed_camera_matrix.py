#!/usr/bin/env python3
"""
ZED X カメラ行列出力スクリプト
ZED SDK 4.x / 5.x 対応

カメラを開いてキャリブレーションパラメータを取得し、
sensor_msgs/CameraInfo 相当の値 (K, D, R, P) を出力する。
zed_wrapper_node の build_camera_info() と同一の解像度・取得方式を使用。
"""

import argparse
import sys

import pyzed.sl as sl

# ZED X 対応解像度 (zed_wrapper_node.cpp の parse_resolution と同一)
RESOLUTIONS = {
    "HD1200": sl.RESOLUTION.HD1200,
    "HD1080": sl.RESOLUTION.HD1080,
    "SVGA":   sl.RESOLUTION.SVGA,
}

# zed_wrapper_node.cpp の kPublishResolution と同一
PUBLISH_WIDTH  = 640
PUBLISH_HEIGHT = 360


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ZED X camera matrix extractor")
    parser.add_argument(
        "--resolution", "-r",
        choices=list(RESOLUTIONS.keys()),
        default="HD1080",
        help="Grab resolution (default: HD1080)",
    )
    parser.add_argument(
        "--publish-width",  type=int, default=PUBLISH_WIDTH,
        help=f"Publish (output) width  (default: {PUBLISH_WIDTH})",
    )
    parser.add_argument(
        "--publish-height", type=int, default=PUBLISH_HEIGHT,
        help=f"Publish (output) height (default: {PUBLISH_HEIGHT})",
    )
    parser.add_argument(
        "--serial-number", "-s", type=int, default=0,
        help="Camera serial number (0 = auto-detect)",
    )
    return parser.parse_args()


def open_camera(args: argparse.Namespace) -> sl.Camera:
    zed = sl.Camera()

    init_params = sl.InitParameters()
    init_params.camera_resolution = RESOLUTIONS[args.resolution]
    init_params.camera_fps        = 30
    init_params.depth_mode        = sl.DEPTH_MODE.NONE
    init_params.coordinate_units  = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
    if args.serial_number != 0:
        init_params.input.set_from_serial_number(args.serial_number)

    ec = zed.open(init_params)
    if ec != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Camera open failed: {ec}")
    return zed


def extract_camera_matrix(zed: sl.Camera, pub_width: int, pub_height: int) -> dict:
    # ネイティブ解像度でintrinsicsを取得し、publish解像度へ手動スケール
    # (Python bindingでget_camera_information(resolution)がスケールしない場合に対応)
    cam_info    = zed.get_camera_information()
    calib       = cam_info.camera_configuration.calibration_parameters.left_cam
    native_res  = cam_info.camera_configuration.resolution

    scale_x = pub_width  / native_res.width
    scale_y = pub_height / native_res.height

    fx = calib.fx * scale_x
    fy = calib.fy * scale_y
    cx = calib.cx * scale_x
    cy = calib.cy * scale_y
    d  = list(calib.disto)  # [k1, k2, p1, p2, k3, ...]

    return {
        "width":  pub_width,
        "height": pub_height,
        "distortion_model": "plumb_bob",
        # D: k1, k2, p1, p2, k3
        "D": [d[0], d[1], d[2], d[3], d[4]],
        # K: row-major 3x3
        "K": [
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0,
        ],
        # R: identity (ZED SDK が rectified 画像を出力するため)
        "R": [1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0],
        # P: row-major 3x4 (stereo baseline なし: Tx=0)
        "P": [
            fx,  0.0, cx,  0.0,
            0.0, fy,  cy,  0.0,
            0.0, 0.0, 1.0, 0.0,
        ],
    }


def print_camera_matrix(params: dict) -> None:
    w, h = params["width"], params["height"]
    D = params["D"]
    K = params["K"]
    R = params["R"]
    P = params["P"]

    print(f"image_width:  {w}")
    print(f"image_height: {h}")
    print(f"distortion_model: {params['distortion_model']}")
    print()
    print(f"# D: [k1, k2, p1, p2, k3]")
    print(f"D: [{', '.join(f'{v:.10f}' for v in D)}]")
    print()
    print(f"# K: 3x3 camera matrix")
    print(f"K: [{K[0]:.6f}, {K[1]:.6f}, {K[2]:.6f},")
    print(f"    {K[3]:.6f}, {K[4]:.6f}, {K[5]:.6f},")
    print(f"    {K[6]:.6f}, {K[7]:.6f}, {K[8]:.6f}]")
    print()
    print(f"# R: 3x3 rectification matrix")
    print(f"R: [{R[0]:.1f}, {R[1]:.1f}, {R[2]:.1f},")
    print(f"    {R[3]:.1f}, {R[4]:.1f}, {R[5]:.1f},")
    print(f"    {R[6]:.1f}, {R[7]:.1f}, {R[8]:.1f}]")
    print()
    print(f"# P: 3x4 projection matrix")
    print(f"P: [{P[0]:.6f}, {P[1]:.6f}, {P[2]:.6f}, {P[3]:.6f},")
    print(f"    {P[4]:.6f}, {P[5]:.6f}, {P[6]:.6f}, {P[7]:.6f},")
    print(f"    {P[8]:.6f}, {P[9]:.6f}, {P[10]:.6f}, {P[11]:.6f}]")


def main() -> None:
    args = parse_args()

    zed = open_camera(args)
    try:
        params = extract_camera_matrix(zed, args.publish_width, args.publish_height)
    finally:
        zed.close()

    print_camera_matrix(params)


if __name__ == "__main__":
    main()
