# AIFormula_private

AIフォーミュラ開発用の非公開レポジトリです．

## コマンドまとめ
### モータ起動
1. CANの設定コマンド
```
cd ~/colcon_ws/src/EC7D_AIformula_Control/launchers/shell/can_bringup.sh
sh can_bringup.sh
ifconfig
```
can_bringup.sh
> #!/bin/sh
> sudo modprobe kvaser_usb
> sudo ip link set can0 type can bitrate 500000
> sudo ip link set can0 up

2. canのメッセージの受信確認（can0に流れているデータを可視化）
`candump can0`
3. ロボットのタイヤをcanで回転させる
```
cangen can0 -g -L 8 -I 210 -D 8000000080000000
```
4. ROS2でcanの通信が行われているか確認
```
ros2 launch ros2_socketcan socket_can_bridge.launch.xml
source ~/.bashrc
ros2 topic list
```

> /from_can_bus, canが流れている状況
> /to_can_bus,     自分で作成したプログラムで通信をするときに使う

5. ジョイスティックコントローラでロボットのタイヤを回転させる方法
```
ros2 launch ros2_socketcan socket_can_bridge.launch.xml
ros2 run roboteq_controller roboteq_controller
ros2 launch teleop_twist_joy teleop-launch.py
```

### ZED X(カメラ)の起動
1.ZED Xの起動
```
ros2 run zed_image_publisher zed_image_publisher
```
2.カメラを抜き差しして変更した場合, デーモン再起動
```
sudo systemctl restart zed_x_daemon
```
3.zed depth viewerやzed sensor viewerで確認する場合
```
cd /usr/local/zed/tools
./ZED_Depth_Viewer
```

### VN-200で使用したコマンド
1.VN-200に実行権限を付与
```
sudo chmod 777 /dev/ttyUSB0
```
2.launchファイルによる起動
```
ros2 launch vectornav vectornav.launch.py
```

もし，上記の2.launchファイルによる起動ができなければ，ターミナルでCtrl + rを押した後に，vectornav.launchと打ち込んでください．またはターミナルで
```
history | grep vectornav.launch
```
