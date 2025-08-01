# 課題7: Bagを扱う（Recording and playing back data）

この課題では、ROS2のbag機能を使用してトピックデータの記録・再生を学習します。

## 学習目標
- ros2 bagを使ったデータ記録方法の習得
- 記録したデータの情報確認
- bagファイルの再生と検証
- 配信頻度やノード情報の確認

## 前提条件

まず、Dockerコンテナにアクセスしてください：

```bash
# macOSホスト上で実行
cd /path/to/b4_ros2
./start_macos.sh

# コンテナにアクセス
docker exec -it b4_ros2 bash
```

## 実行手順

### 1. データ記録の準備

#### PublisherとSubscriberの起動
```bash
# ターミナル1: 学生情報Publisherを実行
# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 run student_info_pubsub student_publisher

# ターミナル2: 基本Publisher
# 新しいターミナルでコンテナにアクセス
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 run task04_basic_pubsub publisher
```

### 2. Bagファイルの記録

#### 基本的な記録
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

# 学生情報トピックを記録
ros2 bag record /student_info

# 基本トピックを記録
ros2 bag record /topic
```

#### 複数トピックの同時記録
```bash
# コンテナ内で実行
# 複数のトピックを同時に記録
ros2 bag record /student_info /topic

# 全てのトピックを記録
ros2 bag record -a
```

#### 高度な記録オプション
```bash
# コンテナ内で実行
# 出力ファイル名を指定
ros2 bag record /student_info -o my_bag_file

# 記録時間を制限（30秒）
ros2 bag record /student_info --duration 30

# 圧縮して記録
ros2 bag record /student_info --compression-mode file
```

### 3. Bagファイルの情報確認

```bash
# コンテナ内で実行
# Bagファイルの詳細情報表示
ros2 bag info rosbag2_xxxx_xx_xx-xx_xx_xx

# 記録されたトピック一覧
ros2 bag info rosbag2_xxxx_xx_xx-xx_xx_xx | grep Topic
```

### 4. Bagファイルの再生

#### 基本的な再生
```bash
# コンテナ内で実行
# Bagファイルを再生
ros2 bag play rosbag2_xxxx_xx_xx-xx_xx_xx

# ループ再生
ros2 bag play rosbag2_xxxx_xx_xx-xx_xx_xx --loop

# 倍速再生
ros2 bag play rosbag2_xxxx_xx_xx-xx_xx_xx --rate 2.0
```

#### 特定のトピックのみ再生
```bash
# コンテナ内で実行
# 学生情報トピックのみ再生
ros2 bag play rosbag2_xxxx_xx_xx-xx_xx_xx --topics /student_info
```

### 5. 再生データの検証

```bash
# コンテナ内で実行
# ターミナル1: Bagファイルを再生
ros2 bag play rosbag2_xxxx_xx_xx-xx_xx_xx --loop

# ターミナル2: Subscriberで受信確認
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 run student_info_pubsub student_subscriber

# ターミナル3: トピック配信頻度確認
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
ros2 topic hz /student_info
```

## 学習ポイント

1. **データ記録**: `ros2 bag record`の使用方法
2. **メタデータ分析**: `ros2 bag info`での情報確認  
3. **データ再生**: `ros2 bag play`とオプション
4. **リアルタイム検証**: 再生中のトピック・ノード確認
5. **配信頻度分析**: `ros2 topic hz`での頻度測定
6. **データ保存**: 長期間のデータアーカイブ

## 実習例：完全なワークフロー

### 1. 記録セッション
```bash
# コンテナ内で実行
# ターミナル1: Publisher起動
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 run student_info_pubsub student_publisher

# ターミナル2: 30秒間記録
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 bag record /student_info --duration 30 -o demo_session

# 記録完了後の確認
ros2 bag info demo_session
```

### 2. 検証セッション  
```bash
# コンテナ内で実行
# ターミナル1: 記録したbagを再生
ros2 bag play demo_session --loop

# ターミナル2: 配信頻度確認
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
ros2 topic hz /student_info

# ターミナル3: Subscriberで受信確認
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 run student_info_pubsub student_subscriber
```

## トラブルシューティング

### 記録が開始されない
```bash
# コンテナ内で実行
# トピックが存在するか確認
ros2 topic list

# Publisherが動作しているか確認
ros2 node list
ros2 topic echo /student_info
```

### 再生時にSubscriberが受信しない
```bash
# コンテナ内で実行  
# 再生ノードが動作しているか確認
ros2 node list | grep rosbag2_player

# トピックが配信されているか確認
ros2 topic echo /student_info --once
```

### ファイルサイズが大きすぎる
```bash
# コンテナ内で実行
# 圧縮オプションを使用
ros2 bag record /student_info --compression-mode file --compression-format zstd

# 記録時間を制限
ros2 bag record /student_info --duration 60
```

### コンテナアクセスの問題
```bash
# macOSホスト上で確認
docker ps | grep b4_ros2

# 起動していない場合
cd /path/to/b4_ros2
./start_macos.sh
```
