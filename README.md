# simtoreal

## 起動手順
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch simulator gazebo_ignition.launch.py
```

## 別ターミナル
```bash
ros2 action send_goal /motor_effort_controller/follow_joint_trajectory   control_msgs/action/FollowJointTrajectory   "{trajectory: {joint_names: ['motor_spin_joint'], points: [{positions: [1.57], time_from_start: {sec: 20}}]}}"
```

- 動作

[![Demo Video](https://img.youtube.com/vi/6M-BkIyV8xc/0.jpg)](https://youtu.be/6M-BkIyV8xc)

## 問題
黄色パーツと赤色パーツの接触によって赤色パーツを動かそうとしているが, 貫通してしまう. 
sdfファイルで同じモデルリンク同士の接触を設定できるが, ros2_controlを使用しているためurdfファイルを用いてロボットモデルをスポーンしている. 

