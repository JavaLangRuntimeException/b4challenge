# ROS2 Python課題実装

このディレクトリには、ROS2 Pythonの課題に対応するパッケージが含まれています。

## 課題一覧

1. **ROS2のノードを動かす** - turtlesimの起動と操作
2. **トピック通信を理解する** - 亀のキーボード操作
3. **ROS2のパッケージを自作し、ビルドする** - カスタムパッケージの作成
4. **簡単なPublisherとSubscriberを書いて実行する** - 基本的な文字列通信
5. **カスタムメッセージを使えるようになる1** - 学生情報メッセージ
6. **カスタムメッセージを使えるようになる2** - 学生情報配列メッセージ
7. **Bagを扱う** - トピックの記録・再生
8. **launchファイルの書き方を理解する** - 複数ノードの同時実行
9. **画像ファイルをトピック通信** - OpenCVとcv_bridgeを使用
11. **2つのマシン間でトピック通信** - マルチコンテナ通信

## 環境セットアップ

### Dev Containersを使用した開発環境（推奨）

1. VS Codeで本リポジトリを開く
2. "Reopen in Container"を選択
3. Dev Container環境が自動で構築される

### 手動でのコンテナ起動

```bash
# WSL2のUbuntu上で実行
cd /path/to/b4_ros2
docker compose up -d
docker exec -it b4_ros2 bash
```

## 課題実行方法

### 課題1-2: turtlesimの動作確認

```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash

# ターミナル1: turtlesimノードの起動
ros2 run turtlesim turtlesim_node

# ターミナル2: キーボード操作ノードの起動
ros2 run turtlesim turtle_teleop_key
```

### 課題3: パッケージ作成とビルド

```bash
# ワークスペースの準備
cd /workspace/src

# パッケージ作成
ros2 pkg create --build-type ament_python my_first_package

# ワークスペースのビルド
cd /workspace
colcon build

# 環境変数の設定
source install/setup.bash
```

### 課題4: 基本的なPublisher/Subscriber

```bash
# ビルド
cd /workspace
colcon build --packages-select task04_basic_pubsub
source install/setup.bash

# ターミナル1: Publisher実行
ros2 run task04_basic_pubsub publisher

# ターミナル2: Subscriber実行
ros2 run task04_basic_pubsub subscriber
```

### 課題5: カスタムメッセージ（学生情報）

```bash
# ビルド
cd /workspace
colcon build --packages-select task05_student_msgs task05_student_info_pubsub
source install/setup.bash

# ターミナル1: Publisher実行
ros2 run task05_student_info_pubsub student_publisher

# ターミナル2: Subscriber実行
ros2 run task05_student_info_pubsub student_subscriber
```

### 課題6: カスタムメッセージ（学生情報配列）

```bash
# ビルド
cd /workspace
colcon build --packages-select task05_student_msgs task06_student_array_pubsub
source install/setup.bash

# ターミナル1: Publisher実行
ros2 run task06_student_array_pubsub student_array_publisher

# ターミナル2: Subscriber実行
ros2 run task06_student_array_pubsub student_array_subscriber
```

### 課題7: Bagの記録と再生

#### Bagファイルの記録
```bash
# 学生情報トピックを記録
ros2 bag record /student_info

# 複数のトピックを記録
ros2 bag record /student_info /topic

# 全てのトピックを記録
ros2 bag record -a

# 特定の時間だけ記録（10秒間）
ros2 bag record /student_info --duration 10

# 出力ディレクトリを指定
ros2 bag record /student_info -o my_bag_data
```

#### Bagファイルの情報確認
```bash
# 記録されたbagの情報確認
ros2 bag info rosbag2_[timestamp]

# 例: 詳細情報表示
ros2 bag info rosbag2_2024_01_01-12_34_56
```

出力例：
```
Files:             rosbag2_2024_01_01-12_34_56_0.db3
Bag size:          1.2 MiB
Storage id:        sqlite3
Duration:          30.045s
Start:             Jan  1 2024 12:34:56.123 (1704088496.123)
End:               Jan  1 2024 12:35:26.168 (1704088526.168)
Messages:          120
Topic information: Topic: /student_info | Type: student_msgs/msg/StudentInfo | Count: 120 | Serialization Format: cdr
```

#### Bagファイルの再生
```bash
# bagの再生
ros2 bag play rosbag2_[timestamp]

# 再生速度を変更（2倍速）
ros2 bag play rosbag2_[timestamp] --rate 2.0

# ループ再生
ros2 bag play rosbag2_[timestamp] --loop

# 特定のトピックのみ再生
ros2 bag play rosbag2_[timestamp] --topics /student_info
```

#### 再生中の確認
```bash
# 再生中のトピック確認
ros2 topic list
ros2 topic echo /student_info

# 配信頻度確認
ros2 topic hz /student_info

# ノード確認
ros2 node list
ros2 node info /rosbag2_player

# トピック詳細確認
ros2 topic info /student_info
```

### 課題8: Launchファイルの実行

```bash
# ビルド
cd /workspace
colcon build --packages-select task08_launch_example
source install/setup.bash

# launchファイルの実行
ros2 launch task08_launch_example student_pubsub_launch.py
```

### 課題9: 画像通信

```bash
# ビルド
cd /workspace
colcon build --packages-select task09_image_pubsub
source install/setup.bash

# ターミナル1: 画像Publisher実行
ros2 run task09_image_pubsub image_publisher

# ターミナル2: 画像Subscriber実行
ros2 run task09_image_pubsub image_subscriber
```

### 課題11: マルチコンテナ通信

#### 環境設定
```bash
# マルチコンテナ環境の起動
docker compose -f docker-compose-multi.yaml up -d

# コンテナの確認
docker ps
```

#### Publisher側（コンテナ1）
```bash
# コンテナ1に接続
docker exec -it ros2_container1 bash

# 環境設定
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Publisherを実行
ros2 run basic_pubsub publisher
# または
ros2 run student_info_pubsub student_publisher
```

#### Subscriber側（コンテナ2）
```bash
# 新しいターミナルでコンテナ2に接続
docker exec -it ros2_container2 bash

# 環境設定
cd /workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Subscriberを実行
ros2 run basic_pubsub subscriber
# または
ros2 run student_info_pubsub student_subscriber
```

#### ネットワーク疎通確認
```bash
# 各コンテナでノード確認
ros2 node list

# トピック確認
ros2 topic list

# ネットワーク設定確認
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY

# 通信確認
ros2 topic echo /topic
```

#### ROS2ネットワーク設定
両コンテナで以下の環境変数が設定されています：
- `ROS_DOMAIN_ID=0`: 同じドメイン番号
- `ROS_LOCALHOST_ONLY=0`: ネットワーク間通信を許可

#### トラブルシューティング
```bash
# ネットワーク確認
docker network ls
docker network inspect b4_ros2_ros2_network

# コンテナ間の接続確認
docker exec ros2_container1 ping ros2_container2
docker exec ros2_container2 ping ros2_container1

# ROS2デーモン再起動
ros2 daemon stop
ros2 daemon start
```

## トラブルシューティング

### GUI表示の問題

```bash
# WSLgの確認（PowerShellで実行）
wsl --version

# Rviz2でGUI表示テスト
rviz2
```

### ビルドエラーの解決

```bash
# 依存関係の更新
cd /workspace
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# クリーンビルド
rm -rf build/ install/ log/
colcon build
```

### ネットワーク設定の確認

```bash
# ROS_DOMAIN_IDの設定
export ROS_DOMAIN_ID=0

# ノード確認
ros2 node list

# トピック確認
ros2 topic list
```

## 参考資料

- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Python API](https://docs.ros.org/en/humble/p/rclpy/)
- [colcon - Collective Construction](https://colcon.readthedocs.io/)
