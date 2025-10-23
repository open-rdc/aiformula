## 準備
自分のgemini apiを取得します.\
(gemini apiは無料で使えます.)\
以下のリンクから作成できます.\
https://aistudio.google.com/api-keys  

---
## 起動手順
```bash
cd ros2_ws
source install/setup.bash
```
```bash
 export GEMINI_API_KEY='自分で作成したAPIを入力'
```
```bash
ros2 launch simulator gazebo_ignition.launch.py gemini_model:=gemini-flash-latest
```

---
## 主なパラメータ
以下のパラメータは  
/gemini_cmd_vel/gemini_cmd_vel/gemini_controller_node.py  
から調整することができます.
```bash
request_interval_sec　   5.0　 #画像をgeminiに送って指示を取得する間隔(秒)(最大4秒まで)　　
control_period_sec       0.1　 #更新されたtwistを再発行する周期(秒)  
max_linear_speed         1.0　 #出力する並進速度の上限(m/s)  
max_angular_speed        1.3   #旋回速度の上限(rad/s)  
max_linear_slew          0.4   #一秒あたりに並進速度をどれだけ変化させるかを決めるスルーレート(m/s^2)  
max_angular_slew         2.0　 #一秒あたりに旋回速度をどれだけ変化させるかを決めるスルーレート(rad/s^2)  
min_linear_speed         0.2　 #並進速度の最低速度(m/s)  
```

---
## プロンプト
geminiに送る文章です.  
/gemini_cmd_vel/gemini_cmd_vel/gemini_controller_node.py  
で編集できます.
```bash
 'base_prompt',
You control a simulated race car. 
Using the provided forward-facing camera image, 
decide speed and steering to stay at the center between two white lane lines and keep moving without stopping. 
Avoid driving onto the green surface areas; remain on the asphalt lane. 
If a white lane line appears ahead on the current heading, adjust steering to stay between the lines and prevent crossing them. 
Respond ONLY with a JSON object containing "linear_x" (m/s) and "angular_z" (rad/s). Positive angular_z turns left.
 Keep absolute values within the supplied speed limits, maintain smooth forward motion, and avoid reversing or oscillating. 
No narration or markdown.
```

---

