# 課題8: Launch ファイルの作成と実行

このパッケージは、複数のROS2ノードを同時に起動するためのlaunchファイルの例を提供します。

## 含まれているlaunchファイル

### 1. basic_pubsub_launch.py
基本的なPublisher/Subscriberを同時に実行します。

### 2. student_pubsub_launch.py
学生情報Publisher/Subscriberを同時に実行します。

### 3. image_pubsub_launch.py
画像Publisher/Subscriberを同時に実行します。

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
colcon build --packages-select launch_example
source install/setup.bash
```

### 2. 実行

#### 基本的なPub/Sub
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 launch launch_example basic_pubsub_launch.py
```

実行結果：
- task04_basic_pubsubのpublisherとsubscriberが同時に起動
- 自動的にメッセージの送受信が開始される

#### 学生情報Pub/Sub
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 launch launch_example student_pubsub_launch.py
```

実行結果：
- student_info_pubsubのpublisherとsubscriberが同時に起動
- 学生情報メッセージの送受信が開始される

#### 画像Pub/Sub
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 launch launch_example image_pubsub_launch.py
```

実行結果：
- image_pubsubのpublisherとsubscriberが同時に起動
- 画像データの送受信が開始される

### 3. Launch実行中の確認

```bash
# 新しいターミナルでコンテナにアクセス
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash

# 起動中のノード確認
ros2 node list

# アクティブなトピック確認
ros2 topic list

# メッセージの確認
ros2 topic echo /topic
ros2 topic echo /student_info
```

## Launchファイルの構造

### 基本的なlaunchファイルの例
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task04_basic_pubsub',
            executable='publisher',
            name='basic_publisher'
        ),
        Node(
            package='task04_basic_pubsub', 
            executable='subscriber',
            name='basic_subscriber'
        )
    ])
```

## 学習ポイント

1. **複数ノード同時起動**: launchファイルでの効率的なシステム起動
2. **ノード間連携**: PublisherとSubscriberの自動的な接続
3. **パラメータ設定**: launchファイルでのパラメータ指定
4. **名前空間**: ノード名やトピック名の管理
5. **条件付き実行**: 環境に応じた柔軟な実行制御

## 高度な使用例

### パラメータ付きでの実行
```bash
# コンテナ内で実行
# パラメータ指定でlaunch実行
ros2 launch launch_example student_pubsub_launch.py use_sim_time:=true

# デバッグモードで実行
ros2 launch launch_example basic_pubsub_launch.py --debug
```

### 特定のノードのみ実行
```bash
# コンテナ内で実行
# launchファイルの一部のみ実行（カスタマイズが必要）
ros2 launch launch_example basic_pubsub_launch.py publisher_only:=true
```

## トラブルシューティング

### Launchファイルが見つからない
```bash
# コンテナ内で実行
# パッケージが正しくビルドされているか確認
ros2 pkg list | grep launch_example

# launchファイルが存在するか確認
ros2 launch launch_example --show-args
```

### ノードが起動しない
```bash
# コンテナ内で実行
# 依存パッケージがビルドされているか確認
ros2 pkg list | grep task04_basic_pubsub
ros2 pkg list | grep student_info_pubsub

# 実行ファイルが存在するか確認
ros2 pkg executables task04_basic_pubsub
```

### 複数ノード間の通信問題
```bash
# コンテナ内で実行
# ノードが正常に起動しているか確認
ros2 node list

# トピックが正しく作成されているか確認
ros2 topic list

# ノード間の接続確認
ros2 node info /basic_publisher
ros2 node info /basic_subscriber
```

### コンテナアクセスの問題
```bash
# macOSホスト上で確認
docker ps | grep b4_ros2

# 起動していない場合
cd /path/to/b4_ros2
./start_macos.sh
```

## 応用課題

1. 独自のlaunchファイルを作成して、3つ以上のノードを同時起動
2. パラメータを使用してノードの動作を制御
3. 条件分岐を使用して環境に応じた起動制御を実装
