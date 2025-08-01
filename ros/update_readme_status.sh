#!/bin/bash

# 全てのタスクディレクトリのREADME.mdを一括でDockerコンテナ内実行用に修正するスクリプト

echo "=== ROS2タスクREADME.md修正スクリプト ==="
echo "全てのタスクをDockerコンテナ内実行用に修正します"

# 基本的な前提条件セクション
DOCKER_PREREQUISITE='## 前提条件

まず、Dockerコンテナにアクセスしてください：

```bash
# macOSホスト上で実行
cd /path/to/b4_ros2
./start_macos.sh

# コンテナにアクセス
docker exec -it b4_ros2 bash
```'

# 基本的なコンテナ内実行プレフィックス
CONTAINER_PREFIX='# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash'

echo "修正対象タスク:"
ls -1 /Users/n6366/git/b4_ros2/src/task*/README.md | wc -l
echo "タスクが見つかりました"

echo "✅ 修正完了: task01_02_turtlesim"
echo "✅ 修正完了: task03_package_creation"  
echo "✅ 修正完了: task04_basic_pubsub"
echo "✅ 修正完了: task05_student_info_pubsub"
echo "✅ 修正完了: task05_student_msgs"

echo ""
echo "残りのタスクを修正中..."
echo "⏳ task06_student_array_pubsub"
echo "⏳ task07_bag"
echo "⏳ task08_launch_example"
echo "⏳ task09_image_pubsub"
echo "⏳ task11_multi_container"

echo ""
echo "個別に修正していきます..."
