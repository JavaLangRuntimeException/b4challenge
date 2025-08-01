#!/bin/bash

# ROS2パッケージの実行テストスクリプト

echo "=== ROS2パッケージ実行テスト ==="

echo "1. コンテナ内でパッケージリストを確認..."
docker exec -it b4_ros2 bash -c "cd /home/n6366/ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 pkg list | grep task"

echo ""
echo "2. 利用可能な実行ファイルを確認..."
docker exec -it b4_ros2 bash -c "cd /home/n6366/ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 pkg executables task04_basic_pubsub"

echo ""
echo "3. パブリッシャーを短時間実行..."
docker exec -it b4_ros2 bash -c "cd /home/n6366/ws && source /opt/ros/humble/setup.bash && source install/setup.bash && timeout 3s ros2 run task04_basic_pubsub publisher || echo 'Publisher test completed'"

echo ""
echo "✅ テスト完了！"
