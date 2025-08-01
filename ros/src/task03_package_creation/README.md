# 課題3: ROS2のパッケージを自作し、ビルドする

この課題では、`ros2 pkg create`と`colcon build`を使ってROS2パッケージの作成とビルドを学習します。

## 学習目標
- ROS2パッケージの作成方法を理解する
- colcon buildシステムの使用方法を習得する
- パッケージの依存関係を理解する

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

### 1. ワークスペースの準備
```bash
# コンテナ内で実行
# ROS2環境をセットアップ
source /opt/ros/humble/setup.bash

# ワークスペースディレクトリに移動
cd ~/ws/src

# 現在のディレクトリ確認
pwd
```

### 2. Pythonパッケージの作成
```bash
# コンテナ内で実行
# Python用パッケージを作成
ros2 pkg create --build-type ament_python my_first_package

# 作成されたディレクトリ構成を確認
tree my_first_package
# または
ls -la my_first_package/
```

### 3. C++パッケージの作成（参考）
```bash
# コンテナ内で実行
# C++用パッケージを作成
ros2 pkg create --build-type ament_cmake my_first_cpp_package

# 依存関係を指定してパッケージを作成
ros2 pkg create --build-type ament_cmake my_cpp_package --dependencies rclcpp std_msgs
```

### 4. 依存関係付きPythonパッケージの作成
```bash
# コンテナ内で実行
# 依存関係を指定してPythonパッケージを作成
ros2 pkg create --build-type ament_python my_python_package --dependencies rclpy std_msgs geometry_msgs
```

### 5. パッケージ内容の確認
```bash
# コンテナ内で実行
# package.xmlの内容確認
cat my_first_package/package.xml

# setup.pyの内容確認
cat my_first_package/setup.py
```

### 6. ビルドの実行
```bash
# コンテナ内で実行
# ワークスペースのルートに移動
cd ~/ws

# 特定のパッケージのみビルド
colcon build --packages-select my_first_package

# 全パッケージのビルド
colcon build

# 並列ビルド（高速化）
colcon build --parallel-workers 4
```

### 7. 環境変数の設定
```bash
# コンテナ内で実行
# ビルド結果の環境変数設定
source install/setup.bash

# 設定の確認
echo $AMENT_PREFIX_PATH
```

### 8. パッケージの確認
```bash
# コンテナ内で実行
# パッケージ一覧表示
ros2 pkg list

# 作成したパッケージの確認
ros2 pkg list | grep my_first_package

# パッケージの詳細情報
ros2 pkg xml my_first_package
```

## パッケージ構成の理解

### Pythonパッケージの基本構成
```
my_first_package/
├── package.xml          # パッケージメタデータ
├── resource/
│   └── my_first_package # リソースマーカー
├── setup.cfg           # セットアップ設定
├── setup.py            # Pythonセットアップスクリプト
├── my_first_package/   # Python実行ファイル格納
│   └── __init__.py
└── test/               # テストファイル
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

### package.xmlの重要な要素
```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_first_package</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <!-- 依存関係の定義 -->
  <depend>rclpy</depend>
  <depend>std_msgs</depend>

  <!-- テスト依存関係 -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <!-- ビルドタイプ -->
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## colcon buildの詳細

### ビルドオプション
```bash
# コンテナ内で実行
# クリーンビルド
cd ~/ws
rm -rf build/ install/ log/
colcon build

# 詳細出力
colcon build --event-handlers console_direct+

# 特定のパッケージのみ
colcon build --packages-select package_name

# 特定のパッケージを除外
colcon build --packages-skip package_name

# 依存関係まで含めてビルド
colcon build --packages-up-to package_name
```

### ビルド結果の確認
```bash
# コンテナ内で実行
# ビルドログの確認
ls -la log/latest_build/

# エラーログの確認
cat log/latest_build/my_first_package/stderr.log

# 成功ログの確認
cat log/latest_build/my_first_package/stdout.log
```

## 実習例：簡単なノードの追加

### 1. Pythonファイルの作成
```bash
# コンテナ内で実行
# ノードファイルを作成
cat > my_first_package/my_first_package/my_node.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_first_node')
        self.get_logger().info('My first node has been started!')

def main(args=None):
    rclpy.init(args=args)
    node = MyFirstNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### 2. setup.pyの更新
```python
# setup.pyのentry_pointsセクションを更新
entry_points={
    'console_scripts': [
        'my_node = my_first_package.my_node:main',
    ],
},
```

### 3. ビルドと実行
```bash
# コンテナ内で実行
# ビルド
cd ~/ws
colcon build --packages-select my_first_package
source install/setup.bash

# 実行
ros2 run my_first_package my_node
```

## 学習ポイント

1. **パッケージ作成**: `ros2 pkg create`コマンドの使用法
2. **ビルドシステム**: colconの基本的な使用方法
3. **依存関係管理**: package.xmlでの依存関係定義
4. **環境変数**: install/setup.bashの重要性
5. **パッケージ構成**: ament_pythonの標準構成
6. **実行ファイル登録**: setup.pyでのentry_points設定

## トラブルシューティング

### ビルドエラーの対処
```bash
# コンテナ内で実行
# 依存関係の更新
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# クリーンビルド
cd ~/ws
rm -rf build/ install/ log/
colcon build

# 詳細なエラー情報
colcon build --event-handlers console_direct+
```

### パッケージが見つからない
```bash
# コンテナ内で実行
# 環境変数の再設定
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

# パッケージパスの確認
echo $AMENT_PREFIX_PATH
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
