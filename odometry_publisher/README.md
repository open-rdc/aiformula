# odometry_publisher
- 当初はホイールエンコーダのみを用いた`wheel_odometry`を作成
- しかし，角度の精度が悪いので，ホイールエンコーダ と Imu (角度情報のみ使用) を組み合わせた`gyro_odometry`を作成
- 精度の良い`gyro_odometry`を使用することを推奨
- `wheel_odometry`も今後のために残しておく

### 使用方法
```sh
$ colcon build --symlink-install --packages-up-to odometry_publisher
$ ros2 launch odometry_publisher gyro_odometry.launch.py
```

### 機能
- `wheel_odometry`
    - Subscribe
        - `/aiformula_sensing/can/frame`
            - CAN情報
            - この中にホイールエンコーダの情報も含まれてる
    - Publish
        - `/aiformula_sensing/wheel_odometry/odom`
            - `odom` to `base_footprint`
    - TF Broadcast
        - `/tf_static`
            - `odom` to `base_footprint`

- `gyro_odometry`
    - Subscribe
        - `/aiformula_sensing/vectornav/imu`
        - `/aiformula_sensing/can/frame`
    - Publish
        - `/aiformula_sensing/gyro_odometry/odom`
    - TF Broadcast
        - `/tf_static`

### ホイールエンコーダー
- id
    - 1409 (0x581): CAN open 指令 (SDO送信)
    - 1537 (0x601): CAN open 指令 (SDO受信)
    - 1809 (0x711): 回転数
