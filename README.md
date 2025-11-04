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

## 問題
<self_collide>タグがtrueのときとfalseのときのモデルの走行速度が変化してしまう. 以下がその様子である. 

- <self_collide>false</self_collide>のとき

[![Demo Video](https://img.youtube.com/vi/FSPZuZA8ILo/0.jpg)](https://youtu.be/FSPZuZA8ILo)

- <self_collide>false</self_collide>のとき

[![Demo Video](https://img.youtube.com/vi/vYH0yDb1K1o/0.jpg)](https://youtu.be/vYH0yDb1K1o)

