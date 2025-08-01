# 課題1-2: ROS2のノードを動かす & トピック通信を理解する

## 前提条件

まず、Dockerコンテナを起動してアクセスしてください：

```bash
# macOSホスト上で実行
cd /path/to/b4_ros2
./start_macos.sh

# コンテナにアクセス
docker exec -it b4_ros2 bash
```

## 課題1: turtlesim_nodeを実行し、亀を表示する

### 実行手順

```bash
# コンテナ内で実行
# ROS2環境のセットアップ
source /opt/ros/humble/setup.bash

# turtlesimノードの実行
ros2 run turtlesim turtlesim_node
```

### 実行結果
- turtlesimのGUIウィンドウが表示される（XQuartzが必要）
- 中央に青い亀が表示される

## 課題2: 亀をキーボードで操作する

### 実行手順

```bash
# ターミナル1: turtlesimノード実行（課題1から継続）
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node

# ターミナル2: キーボード操作ノード実行（新しいターミナルで）
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtle_teleop_key
```

### 操作方法
- `i`: 前進
- `j`: 左回転
- `l`: 右回転
- `k`: 後退
- `u`, `o`: 斜め移動
- `m`, `.`: 斜め後退移動

### 学習内容

#### トピック通信の確認
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash

# トピック一覧表示
ros2 topic list

# 亀の位置情報確認
ros2 topic echo /turtle1/pose

# 速度コマンド確認
ros2 topic echo /turtle1/cmd_vel

# トピックの詳細情報
ros2 topic info /turtle1/cmd_vel
ros2 topic info /turtle1/pose
```

#### ノード情報の確認
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash

# ノード一覧表示
ros2 node list

# ノードの詳細情報
ros2 node info /turtlesim
ros2 node info /teleop_turtle
```

#### 手動でのコマンド送信
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash

# 手動で亀を動かす
ros2 topic pub /turtle1/cmd_vel geometry_msgs/Twist "linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}"

# 一度だけコマンド送信
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/Twist "linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

#### rqtを使ったグラフィカル確認
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash

# rqt_graph でノード関係図を表示
ros2 run rqt_graph rqt_graph

# rqt_plot で位置データをプロット
ros2 run rqt_plot rqt_plot
```

## 使用されるメッセージ型

### geometry_msgs/Twist
亀の速度制御に使用
```
geometry_msgs/Vector3 linear
  float64 x
  float64 y  
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z
```

### turtlesim/Pose  
亀の位置情報
```
float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity
```

## 学習ポイント

1. **ROS2ノードの実行**: `ros2 run`コマンドの使用
2. **トピック通信**: Publisher/Subscriberの概念
3. **メッセージ型**: geometry_msgs/Twist, turtlesim/Poseの理解
4. **リアルタイム制御**: キーボード入力による即座の反応
5. **GUIアプリケーション**: WSLgでのGUI表示確認
6. **コマンドラインツール**: ros2 topic, ros2 nodeの活用

## トラブルシューティング

### GUIが表示されない
```bash
# macOSホスト上で確認
# XQuartzが起動しているか確認
ps aux | grep -i xquartz

# X11転送を許可
xhost +localhost

# コンテナ内でテスト
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
echo $DISPLAY
rviz2
```

### キー操作が効かない
- teleop_turtleを実行したターミナルがアクティブになっていることを確認
- キーを押しても反応がない場合は、Ctrl+Cで終了して再実行

### Dockerコンテナが起動していない
```bash
# macOSホスト上で確認・起動
cd /path/to/b4_ros2
docker ps | grep b4_ros2

# 起動していない場合
./start_macos.sh
```
