# docker

AIFormulaの開発環境を構築するための Docker 環境です。

## 前提

- [Docker](https://docs.docker.com/get-docker/) がインストール済みであること
- GPU を使う場合は [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) がインストール済みであること

## 使い方

以下のスクリプトはすべて `docker/` ディレクトリ内で実行してください。

### 1. イメージのビルド

```bash
./build.sh
```

`aiformula:humble` というイメージが作成されます。

### 2. コンテナの起動

```bash
# GPU を使わない場合
./run.sh

# GPU を使う場合
./run_gpu.sh
```

`my-aiformula-humble` という名前でコンテナが起動し、そのまま bash に入ります。
（GUIが表示されない場合はホスト側で `xhost +local:docker` を実行してください）。

### 3. ワークスペースのビルド（初回のみ）

コンテナ内で以下を実行します。

```bash
cd ~/formula_ws
cb        # colcon build --symlink-install のエイリアス
source ~/.bashrc
```

### コンテナへの再接続

`exit` するとコンテナは停止します。再度入る場合は次のようにします。

```bash
# 停止中のコンテナを起動して入る
docker start -i my-aiformula-humble

# 起動中のコンテナに別シェルから入る
docker exec -it my-aiformula-humble bash
```

## 補足

### シェルの便利機能

#### bash（エイリアス / 関数）

| コマンド | 内容 |
| --- | --- |
| `cb` | `colcon build --symlink-install` |
| `cbcl` | `install/ build/ log/` を削除してからクリーンビルド |
| `bashrc` | `~/.bashrc` を再読み込み |
| `ros_make` | ワークスペースへ移動してビルドし、元のディレクトリへ戻る |

- プロンプトにカレントディレクトリの git ブランチを表示します。
- 起動時に `/opt/ros/humble/setup.bash` と `~/formula_ws/install/setup.bash` を自動で source します。
- `ROS_DOMAIN_ID=10` を設定済みです。

#### tmux

| 操作 | キー |
| --- | --- |
| prefix | `C-a`（`C-b` から変更） |
| ペインを垂直分割 | `prefix` + `\` |
| ペインを水平分割 | `prefix` + `-` |
| ペイン移動 | `C-h` / `C-j` / `C-k` / `C-l`（`C-o` で順送り） |
| ペインのリサイズ | `prefix` + `H` / `J` / `K` / `L` |
| ウィンドウ切り替え | `Shift` + `←` / `→` |
| 設定のリロード | `prefix` + `r` |

- マウス操作が有効です。
- ステータスラインは 256 色対応、ウィンドウ一覧は右寄せ表示です。

#### vim

- 行番号を表示します（`set number`）。

### ホストとのファイル共有

`docker/docker_share/` がコンテナ内の `/home/host_files` にマウントされます。
ホストとコンテナの間でファイルをやり取りしたいときはこのディレクトリを使ってください。