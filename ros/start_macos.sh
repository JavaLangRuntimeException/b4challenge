#!/bin/bash

# macOS用ROS2環境起動スクリプト

echo "=== ROS2 Docker環境の起動（macOS用） ==="

# XQuartzがインストールされているか確認
if ! command -v xquartz &> /dev/null && ! ls /Applications/ | grep -i xquartz &> /dev/null; then
    echo "⚠️  XQuartzがインストールされていません。"
    echo "以下のコマンドでインストールしてください："
    echo "  brew install --cask xquartz"
    echo "または https://www.xquartz.org/ からダウンロードしてください。"
    echo ""
fi

# XQuartzの起動を試行
if ls /Applications/ | grep -i xquartz &> /dev/null; then
    if ! pgrep -x "Xquartz" > /dev/null; then
        echo "XQuartzを起動しています..."
        open -a XQuartz 2>/dev/null || echo "⚠️ XQuartzの起動に失敗しました。手動で起動してください。"
        sleep 3
    else
        echo "✅ XQuartzは既に起動しています。"
    fi
fi

# X11転送の許可（xhostがある場合のみ）
if command -v xhost &> /dev/null; then
    echo "X11転送を許可しています..."
    xhost +localhost
else
    echo "⚠️ xhostコマンドが見つかりません。XQuartzをインストールしてください。"
fi

# Dockerが起動しているか確認
if ! docker info > /dev/null 2>&1; then
    echo "❌ Dockerが起動していません。Docker Desktopを起動してください。"
    exit 1
fi

# 現在のユーザーIDとグループIDを.envファイルに設定
echo "USER_NAME=${USER}" > .env
echo "GROUP_NAME=${USER}" >> .env
echo "UID=$(id -u)" >> .env
echo "GID=$(id -g)" >> .env
echo "ROS_DOMAIN_ID=1" >> .env
echo "ROS_LOCALHOST_ONLY=1" >> .env

echo "環境変数を設定しました:"
cat .env

# Dockerコンテナのビルドと起動
echo "Dockerコンテナをビルド・起動しています..."
docker compose up -d --build

# コンテナが起動するまで待機
echo "コンテナの起動を待っています..."
sleep 5

# 起動確認
if docker ps | grep b4_ros2 > /dev/null; then
    echo "✅ ROS2コンテナが正常に起動しました！"
    echo ""
    echo "次のコマンドでコンテナにアクセスできます："
    echo "  docker exec -it b4_ros2 bash"
    echo ""
    echo "GUI表示のテスト："
    echo "  docker exec -it b4_ros2 bash -c 'source /opt/ros/humble/setup.bash && rviz2'"
    echo ""
    echo "turtlesimのテスト："
    echo "  docker exec -it b4_ros2 bash -c 'source /opt/ros/humble/setup.bash && ros2 run turtlesim turtlesim_node'"
else
    echo "❌ コンテナの起動に失敗しました。"
    echo "ログを確認してください："
    echo "  docker compose logs"
fi
