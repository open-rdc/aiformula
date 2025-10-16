**準備**\
自分のgemini apiの環境設定をします\
(gemini apiは無料で使えます)\
以下のリンクから作成できます\
https://aistudio.google.com/api-keys  
\
**起動手順**  
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
