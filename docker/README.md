# aiformula2026_docker
これはAIForemulaにおける環境構築のためのDockerfileです。
こちらを使用する前にDockerをインストールしてください。
以下が手順になります。

## 手順
```bash
#ホスト側
git clone https://github.com/open-rdc/aiformula
cd docker
docker build -t <your-image-name> .
#例
docker run -it -d --name <your_container_name> <your_image_name>
docker exec -it <your_container_name> /bin/bash
#コンテナ内
cd ~/ros2_ws
colcon build
source ~/.bashrc
```
## GPUをお使いの場合
GPUを使っており、cudaやcuDNNを利用したい場合は以下のサイトを参考にベースイメージ名を変更してください。

- [参考サイト](https://hub.docker.com/r/nvidia/cuda/)

GPUをお使いの場合の実行例は以下のようになります。
```bash
docker run -it \
  --net=host \
  --ipc=host \
  --pid=host \
  --gpus all \
  --name <your_container_name> \
  -e DISPLAY=$DISPLAY \
  -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
  -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
  -e PULSE_SERVER=$PULSE_SERVER \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -v $HOME/.Xauthority:/tmp/.docker.xauth \
  -v /mnt/wslg:/mnt/wslg \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(realpath <your_workspace>):/home/<yourname>/<your_workspace> \
  <your_image_name>

xhost +local:docker > /dev/null && docker start <your_container_name> > /dev/null && docker exec -it <your_container_name> bash
```
