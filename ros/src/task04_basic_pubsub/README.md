# 課題4: 基本的なPublisher/Subscriber

このパッケージは、ROS2の基本的なPublisher/Subscriberの実装例です。

## 機能

- **Publisher**: ユーザーからの入力を受け取り、文字列メッセージとしてPublishします
- **Subscriber**: Publishされた文字列メッセージを受信して表示します

## ファイル構成

```
task04_basic_pubsub/
├── task04_basic_pubsub/
│   ├── __init__.py
│   ├── publisher.py      # Publisher実装
│   └── subscriber.py     # Subscriber実装
├── resource/
│   └── task04_basic_pubsub
├── package.xml           # パッケージ定義
├── setup.py             # セットアップスクリプト
└── README.md           # このファイル
```

## 前提条件

まず、Dockerコンテナにアクセスしてください：

```bash
# macOSホスト上で実行
cd /path/to/b4_ros2
./start_macos.sh

# コンテナにアクセス
docker exec -it b4_ros2 bash
```

## 使用方法

### 1. ビルド

```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
cd ~/ws
colcon build --packages-select task04_basic_pubsub
source install/setup.bash
```

### 2. 実行

#### ターミナル1: Publisher実行
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 run task04_basic_pubsub publisher
```

実行後、メッセージの入力が求められます：
```
Message 0: Hello ROS2!
Message 1: This is a test message
```

#### ターミナル2: Subscriber実行
```bash
# 新しいターミナルでコンテナにアクセス
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 run task04_basic_pubsub subscriber
```

Publisherから送信されたメッセージが表示されます：
```
[INFO] [1234567890.123456789] [minimal_subscriber]: I heard: "[0] Hello ROS2!"
Received message: [0] Hello ROS2!
[INFO] [1234567890.123456789] [minimal_subscriber]: I heard: "[1] This is a test message"
Received message: [1] This is a test message
```

### 3. トピック確認

```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

# トピック一覧表示
ros2 topic list

# トピックの詳細情報
ros2 topic info /topic

# メッセージの監視
ros2 topic echo /topic
```

## 学習ポイント

1. **ROS2ノードの基本構造**: `rclpy.node.Node`クラスの継承
2. **Publisherの作成**: `create_publisher()`メソッドの使用
3. **Subscriberの作成**: `create_subscription()`メソッドの使用
4. **メッセージ型**: `std_msgs.msg.String`の使用
5. **タイマーコールバック**: 定期的な処理の実装
6. **メッセージ送受信**: `publish()`メソッドとコールバック関数

## トラブルシューティング

### Publisherで入力が受け付けられない
- Ctrl+Cで一度終了し、再実行してください
- 端末がフォアグラウンドにあることを確認してください

### メッセージが届かない
```bash
# コンテナ内で実行
# ノード確認
ros2 node list

# トピック確認
ros2 topic list
ros2 topic echo /topic
```

### コンテナアクセスの問題
```bash
# macOSホスト上で確認
# コンテナが起動しているか確認
docker ps | grep b4_ros2

# 起動していない場合
cd /path/to/b4_ros2
./start_macos.sh
```
