# simtoreal

## 起動手順
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch simulator gazebo_ignition.launch.py
```

## 別ターミナル
```bash
ros2 topic pub /motor_spin_angle std_msgs/msg/Float64 "data: 司令角度"
```

- motor_spin(Yellow)とcaster_yaw(Red)との衝突の様子動画

[![Demo Video](https://img.youtube.com/vi/o7riwcc6SNo/0.jpg)](https://youtu.be/o7riwcc6SNo)

- 新従動輪のモータ制御及びモータによる走行中の方向転換

[![Demo_video](https://img.youtube.com/vi/7gYfWRK6dkg/0.jpg)](https://youtu.be/7gYfWRK6dkg) 

- 走行速度の問題は解決した
- <self_collide>false</self_collide>のとき

[![Demo Video](https://img.youtube.com/vi/dlR26O8A4JA/0.jpg)](https://youtu.be/dlR26O8A4JA)

- <self_collide>true</self_collide>のとき

[![Demo Video](https://img.youtube.com/vi/I8MrG4bpj240.jpg)](https://youtu.be/I8MrG4bpj24)
