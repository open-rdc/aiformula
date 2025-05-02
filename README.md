### Install
```
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
````

### Execute

#### Provided Caster
```
ros2 launch simulator gazebo_simulator_caster.launch.py
cd ~/ros2_ws/src/aiformula/simulator/simulator/simulator
./line_follower.py
```

#### Extended-Trail Caster
```
ros2 launch simulator gazebo_simulator_extended_trail.launch.py
cd ~/ros2_ws/src/aiformula/simulator/simulator/simulator
./line_follower.py
```

#### Active Caster
```
ros2 launch simulator gazebo_simulator_active_caster.launch.py
cd ~/ros2_ws/src/aiformula/simulator/simulator/simulator
./caster_controller_sim.py
./line_follower.py
```

![image](https://github.com/user-attachments/assets/3355e190-361a-48b0-9b94-667a8e669210)
