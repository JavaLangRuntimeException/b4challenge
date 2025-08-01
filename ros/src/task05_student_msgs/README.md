# task05_student_msgs パッケージ

ROS 2カスタムメッセージ定義パッケージです。学生情報を送受信するためのメッセージ型を定義します。

## 参考資料
このパッケージは「Creating custom msg and srv files」チュートリアルに基づいて作成されています。

## 概要

このパッケージでは以下の2つのカスタムメッセージを定義しています：
- **StudentInfo.msg**: 単一の学生情報（課題5用）
- **StudentArray.msg**: 複数の学生情報をまとめた配列（課題6用）

## パッケージ構造

```
task05_student_msgs/
├── CMakeLists.txt          # インターフェース生成設定
├── package.xml             # パッケージ依存関係
├── msg/
│   ├── StudentInfo.msg     # 課題5: 単一学生情報
│   └── StudentArray.msg    # 課題6: 学生情報配列
└── README.md
```

## メッセージ定義

### StudentInfo.msg（課題5用）
```
# 学生証番号 (数値)
int32 student_id

# 名前 (文字列)
string name

# 画像データ (sensor_msgs/Image)
sensor_msgs/Image image
```

**機能**: 学生証番号（数値）、名前（文字列）、画像を送ることができるメッセージ

### StudentArray.msg（課題6用）
```
# 学生情報の配列
task05_student_msgs/StudentInfo[] students

# 配列の総数
int32 total_count

# メッセージのタイムスタンプ
std_msgs/Header header
```

**機能**: 学生情報を複数作成し、それらを1つにまとめて送受信できるメッセージ（動的配列を利用）

## 前提条件

ROS 2ワークスペースが準備されている必要があります。

## ビルド方法

### 1. メッセージパッケージのビルド
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
cd ~/ws
colcon build --packages-select task05_student_msgs
source install/setup.bash
```

### 2. インターフェース作成の確認

ビルド後、以下のコマンドでメッセージが正しく作成されたことを確認できます：

```bash
# StudentInfo メッセージの確認
ros2 interface show task05_student_msgs/msg/StudentInfo

# StudentArray メッセージの確認
ros2 interface show task05_student_msgs/msg/StudentArray
```

**期待される出力**:
```
ros2 interface show task05_student_msgs/msg/StudentInfo
int32 student_id
string name
sensor_msgs/Image image
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
        uint32 height
        uint32 width
        string encoding
        uint8 is_bigendian
        uint32 step
        uint8[] data
```

## 他のパッケージでの使用方法

このメッセージパッケージを他のPythonパッケージで使用する場合：

### Python での使用例
```python
from task05_student_msgs.msg import StudentInfo, StudentArray

# StudentInfo メッセージ作成
student_info = StudentInfo()
student_info.student_id = 20240001
student_info.name = "Tanaka Taro"
# student_info.image は sensor_msgs/Image 型

# StudentArray メッセージ作成
student_array = StudentArray()
student_array.students = [student_info]
student_array.total_count = 1
```

### package.xml での依存関係設定
```xml
<depend>task05_student_msgs</depend>
```

### setup.py での依存関係設定（Python パッケージの場合）
```python
install_requires=[
    'setuptools',
    'task05_student_msgs',
],
```

## トラブルシューティング

### ビルドエラーが発生する場合
```bash
# 依存関係をインストール
cd ~/ws
rosdep install --from-paths src --ignore-src -r -y

# クリーンビルド
rm -rf build/ install/ log/
colcon build --packages-select task05_student_msgs
```

### メッセージが見つからない場合
```bash
# 環境変数を再設定
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

# パッケージが正しくインストールされているか確認
ros2 pkg list | grep task05_student_msgs
```

### インターフェースが認識されない場合
```bash
# インターフェース一覧を確認
ros2 interface list | grep task05_student_msgs

# 具体的なメッセージ構造を確認
ros2 interface show task05_student_msgs/msg/StudentInfo
ros2 interface show task05_student_msgs/msg/StudentArray
```

## 学習のポイント

1. **独立したメッセージパッケージ**: カスタムメッセージは専用のament_cmakeパッケージで定義
2. **rosidl_generate_interfaces**: メッセージ定義をC++/Python用コードに変換
3. **依存関係管理**: 他のメッセージ型を使用する場合の適切な依存関係設定
4. **動的配列**: `[]`を使った可変長配列の定義方法
5. **メッセージ合成**: 他のメッセージパッケージのメッセージを組み込む方法
