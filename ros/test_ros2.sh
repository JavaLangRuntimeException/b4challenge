#!/bin/bash

# ROS2環境テストスクリプト

echo "=== ROS2環境テスト ==="

# コンテナが起動しているか確認
if ! docker ps | grep b4_ros2 > /dev/null; then
    echo "❌ b4_ros2コンテナが起動していません。"
    echo "まず ./start_macos.sh を実行してください。"
    exit 1
fi

echo "✅ コンテナは起動中です。"

# ROS2パッケージのテスト
echo ""
echo "🔍 ROS2パッケージをテスト中..."
docker exec -it b4_ros2 bash -c "source /opt/ros/humble/setup.bash && ros2 pkg list | grep turtlesim"

# ワークスペースの確認
echo ""
echo "📁 ワークスペースの構造:"
docker exec -it b4_ros2 bash -c "ls -la /home/n6366/ws/"

# srcディレクトリの確認
echo ""
echo "📂 srcディレクトリの内容:"
docker exec -it b4_ros2 bash -c "ls -la /home/n6366/ws/src/"

# ROSコマンドのテスト
echo ""
echo "🤖 ROS2ノードリストのテスト:"
docker exec -it b4_ros2 bash -c "source /opt/ros/humble/setup.bash && timeout 3s ros2 node list || echo 'ノードリスト取得完了'"

echo ""
echo "✅ 基本テスト完了！"
echo ""
echo "次のステップ:"
echo "1. コンテナにアクセス: docker exec -it b4_ros2 bash"
echo "2. 環境設定: source /opt/ros/humble/setup.bash"
echo "3. turtlesimテスト: ros2 run turtlesim turtlesim_node"
