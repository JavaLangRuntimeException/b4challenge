# 課題11: 2つのマシン間でトピック通信

この課題では、2つのDockerコンテナを使用してマルチマシン間でのROS2トピック通信を実現します。

## 学習目標
- マルチマシン環境でのROS2通信の理解
- Docker Composeを使った複数コンテナの管理
- ROS2ネットワーク設定の理解
- コンテナ間通信の確認

## 前提条件

### ネットワーク設定の理解
ROS2のマルチマシン通信には以下の設定が重要です：
- `ROS_DOMAIN_ID`: 同じ値にすることで通信ドメインを統一
- `ROS_LOCALHOST_ONLY`: 0に設定してネットワーク間通信を許可

## 実行手順

### 1. マルチコンテナ環境の起動

#### docker-compose-multi.yamlの確認
```bash
# macOSホスト上で実行
cd /path/to/b4_ros2
cat docker-compose-multi.yaml
```

#### 環境の起動
```bash
# macOSホスト上で実行
cd /path/to/b4_ros2

# マルチコンテナ環境を起動
docker compose -f docker-compose-multi.yaml up -d

# コンテナの起動確認
docker ps
```

期待される出力：
```
CONTAINER ID   IMAGE       COMMAND       CREATED          STATUS          PORTS     NAMES
abc123def456   b4_ros2     "/bin/bash"   10 seconds ago   Up 9 seconds              ros2_container1
def456abc789   b4_ros2     "/bin/bash"   10 seconds ago   Up 9 seconds              ros2_container2
```

### 2. ネットワーク疎通の確認

#### コンテナ間の接続確認
```bash
# macOSホスト上で実行
# Container1にアクセス
docker exec -it ros2_container1 bash

# Container1内で実行
# Container2への疎通確認
ping ros2_container2

# Container2にアクセス（別ターミナル）
docker exec -it ros2_container2 bash

# Container2内で実行
# Container1への疎通確認
ping ros2_container1
```

### 3. ROS2環境のセットアップ

#### Container1での設定
```bash
# Container1内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

# 環境変数の設定
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0

# 設定確認
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
```

#### Container2での設定
```bash
# Container2内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

# 環境変数の設定（Container1と同じ設定）
export ROS_DOMAIN_ID=1
export ROS_LOCALHOST_ONLY=0

# 設定確認
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
```

### 4. マルチマシン通信のテスト

#### Container1でPublisher実行
```bash
# Container1内で実行
# 基本Publisher
ros2 run task04_basic_pubsub publisher

# または学生情報Publisher
ros2 run student_info_pubsub student_publisher
```

#### Container2でSubscriber実行
```bash
# Container2内で実行
# 基本Subscriber
ros2 run task04_basic_pubsub subscriber

# または学生情報Subscriber
ros2 run student_info_pubsub student_subscriber
```

#### 通信確認
```bash
# Container1とContainer2の両方で実行
# ノード一覧の確認（他のコンテナのノードが見えるはず）
ros2 node list

# トピック一覧の確認
ros2 topic list

# 特定トピックのメッセージ確認
ros2 topic echo /topic
ros2 topic echo /student_info
```

### 5. 双方向通信のテスト

#### Container1でPublisher/Subscriber
```bash
# Container1内で実行
# Publisherを起動
ros2 run task04_basic_pubsub publisher &

# Subscriberも起動
ros2 run task04_basic_pubsub subscriber
```

#### Container2でSubscriber/Publisher
```bash
# Container2内で実行
# Subscriberを起動
ros2 run task04_basic_pubsub subscriber &

# Publisherも起動
ros2 run task04_basic_pubsub publisher
```

## 高度な検証

### ネットワーク遅延の測定
```bash
# Container1内で実行
# タイムスタンプ付きメッセージの送信
ros2 topic pub /test_topic std_msgs/String "data: 'Test message at $(date)'" --once

# Container2内で実行
# メッセージの受信確認
ros2 topic echo /test_topic --once
```

### 大容量データの通信テスト
```bash
# Container1内で実行
# 画像データの送信
ros2 run image_pubsub image_publisher

# Container2内で実行
# 画像データの受信
ros2 run image_pubsub image_subscriber
```

### Discovery確認
```bash
# 両コンテナで実行
# ROS2 Discovery情報の確認
ros2 daemon stop
ros2 daemon start
ros2 node list
```

## 学習ポイント

1. **マルチマシン通信**: 複数コンテナでのROS2通信設定
2. **ネットワーク設定**: ROS_DOMAIN_IDとROS_LOCALHOST_ONLYの理解
3. **Discovery機能**: ROS2の自動ノード発見メカニズム
4. **コンテナオーケストレーション**: Docker Composeでの複数コンテナ管理
5. **ネットワークトラブルシューティング**: 通信問題の診断方法

## トラブルシューティング

### ノードが相互に発見されない
```bash
# 両コンテナで実行
# ROS設定の確認
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY

# ネットワーク設定の確認
ifconfig
route -n
```

### メッセージが届かない
```bash
# 両コンテナで実行
# ファイアウォール設定の確認（必要に応じて）
# DDS設定の確認
ros2 doctor

# 通信ポートの確認
netstat -an | grep 7400
```

### Container間のping不通
```bash
# macOSホスト上で実行
# Docker networkの確認
docker network ls
docker network inspect b4_ros2_default

# コンテナ情報の確認
docker inspect ros2_container1 | grep IPAddress
docker inspect ros2_container2 | grep IPAddress
```

### 環境の再起動
```bash
# macOSホスト上で実行
# 全コンテナの停止・削除
docker compose -f docker-compose-multi.yaml down

# 再起動
docker compose -f docker-compose-multi.yaml up -d
```

## 応用課題

1. 3台以上のコンテナでの通信ネットワーク構築
2. 異なるROS_DOMAIN_IDでの通信分離テスト
3. コンテナの動的追加・削除での通信継続性確認
4. 帯域制限下でのメッセージ通信性能測定

## 実際のマルチマシン環境への応用

この課題で学んだ設定は、実際の複数PCでのROS2システム構築にも応用できます：

1. 各PCで同じROS_DOMAIN_IDを設定
2. ROS_LOCALHOST_ONLY=0に設定
3. ネットワーク疎通を確認
4. ファイアウォール設定の調整（必要に応じて）
