# 課題5: カスタムメッセージを使った学生情報通信

## 概要
「Creating custom msg and srv files」チュートリアルに基づき、カスタムメッセージ`StudentInfo`を使った学生情報通信を実装します。

**課題内容**: 学生証番号（数値），名前（文字列），画像を送ることができるメッセージを作成し、Publisher/Subscriberで送受信を確認する。

## 機能

- **StudentInfo Message**: 学生証番号(int32)、名前(string)、画像(sensor_msgs/Image)を含むカスタムメッセージ
- **Student Publisher**: サンプル学生データを生成してPublish
- **Student Subscriber**: 学生情報を受信して表示・画像保存

## パッケージ構造

```
student_info_pubsub/
├── student_info_pubsub/
│   ├── __init__.py
│   ├── student_publisher.py    # 学生情報Publisher
│   └── student_subscriber.py   # 学生情報Subscriber
├── resource/
│   └── student_info_pubsub
├── package.xml                 # task05_student_msgsに依存
├── setup.py                   # エントリーポイント設定
└── README.md
```

## 依存関係

### 前提パッケージ
- **task05_student_msgs**: カスタムメッセージ定義パッケージ（先にビルドが必要）
- **cv_bridge**: OpenCVとROS2の画像変換
- **opencv-python**: 画像処理ライブラリ

### カスタムメッセージ
```python
# task05_student_msgs/msg/StudentInfo.msg
int32 student_id          # 学生証番号
string name              # 学生名
sensor_msgs/Image image  # 画像データ
```

## セットアップ手順

### 1. 環境起動（macOS）
```bash
# リポジトリルートで実行
cd ~/develop/b4challenge/ros
./start_macos.sh

# コンテナにアクセス
docker exec -it b4_ros2 bash
```

### 2. ビルド（コンテナ内）

**重要**: メッセージパッケージを先にビルドしてからPub/Subパッケージをビルドする

```bash
# ROS 2環境をセットアップ
source /opt/ros/humble/setup.bash
cd ~/ws

# 1. まずメッセージパッケージをビルド
colcon build --packages-select task05_student_msgs
source install/setup.bash

# 2. 次にPub/Subパッケージをビルド
colcon build --packages-select student_info_pubsub
source install/setup.bash
```

### 3. インターフェース確認
```bash
# カスタムメッセージが正しく認識されているか確認
ros2 interface show task05_student_msgs/msg/StudentInfo

# 期待される出力:
# int32 student_id
# string name
# sensor_msgs/Image image
#         ... (Image型の詳細構造)
```

## 実行方法

### ターミナル1: Publisher実行
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
python3 src/task05_student_info_pubsub/student_info_pubsub/student_publisher.py
```

**期待される出力**:
```
[INFO] [student_publisher]: Student Publisher node has been started.
[INFO] [student_publisher]: Publishing student info: ID=20240001, Name="Tanaka Taro", Image size=(200, 300, 3)
[INFO] [student_publisher]: Publishing student info: ID=20240002, Name="Sato Hanako", Image size=(200, 300, 3)
...
```

### ターミナル2: Subscriber実行
```bash
# 新しいターミナルでコンテナにアクセス
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
python3 src/task05_student_info_pubsub/student_info_pubsub/student_subscriber.py
```

**期待される出力**:
```
[INFO] [student_subscriber]: Student Subscriber node has been started.
[INFO] [student_subscriber]: Waiting for student info messages...

=== Student Information #1 ===
Student ID: 20240001
Name: Tanaka Taro
Image size: 300x200
Image encoding: bgr8
OpenCV image shape: (200, 300, 3)
Image saved as: received_student_20240001_1.jpg
========================================
```

### トピック確認
```bash
# 新しいターミナルで確認
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

# トピック一覧
ros2 topic list

# 学生情報トピックの詳細
ros2 topic info /student_info
ros2 topic echo /student_info --once
```

## 実装のポイント

### Python コードでのカスタムメッセージ使用

**Publisher側**:
```python
from task05_student_msgs.msg import StudentInfo
from sensor_msgs.msg import Image

# StudentInfo メッセージ作成
student_msg = StudentInfo()
student_msg.student_id = 20240001
student_msg.name = "Tanaka Taro"
student_msg.image = self.cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")

# トピックに送信
self.publisher_.publish(student_msg)
```

**Subscriber側**:
```python
from task05_student_msgs.msg import StudentInfo

def topic_callback(self, msg):
    # カスタムメッセージからデータ取得
    student_id = msg.student_id
    name = msg.name
    cv_image = self.cv_bridge.imgmsg_to_cv2(msg.image, "bgr8")
```

### package.xml での依存関係設定
```xml
<depend>task05_student_msgs</depend>
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>cv_bridge</depend>
```

### setup.py でのエントリーポイント設定
```python
entry_points={
    'console_scripts': [
        'student_publisher = student_info_pubsub.student_publisher:main',
        'student_subscriber = student_info_pubsub.student_subscriber:main',
    ],
},
```

## 送信されるサンプルデータ

Publisher は以下の学生データを順次送信します：
- ID: 20240001, Name: "Tanaka Taro"
- ID: 20240002, Name: "Sato Hanako"
- ID: 20240003, Name: "Suzuki Ichiro"
- ID: 20240004, Name: "Watanabe Yuki"
- ID: 20240005, Name: "Ito Kazuko"

各学生データには300x200のカラー画像が含まれます。

## 生成されるファイル

Subscriberは受信した画像を以下の形式で保存します：
```
received_student_{学生ID}_{受信回数}.jpg
```

例: `received_student_20240001_1.jpg`, `received_student_20240002_2.jpg`

## トラブルシューティング

### メッセージパッケージが見つからない
```bash
# メッセージパッケージを先にビルド
cd ~/ws
colcon build --packages-select task05_student_msgs
source install/setup.bash

# その後Pub/Subパッケージをビルド
colcon build --packages-select student_info_pubsub
source install/setup.bash
```

### ImportError: No module named 'task05_student_msgs'
```bash
# 環境変数を確認
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

# メッセージパッケージがビルドされているか確認
ros2 pkg list | grep task05_student_msgs
```

### cv_bridgeエラー
```bash
# OpenCVライブラリ確認
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"

# 必要に応じて再インストール
pip3 install opencv-python
```

### GUI表示エラー（GUI環境なしでは正常）
```
Cannot display image (no GUI?): OpenCV(4.5.4) ./modules/highgui/src/window_gtk.cpp:635
```
→ これは正常です。Docker環境では画像表示はできませんが、ファイル保存は正常に動作します。

## 学習のポイント

1. **カスタムメッセージ定義**: 独立したament_cmakeパッケージでの.msg定義
2. **ビルド順序**: メッセージパッケージ → 使用パッケージの順序でビルド
3. **依存関係管理**: package.xmlとsetup.pyでの適切な依存関係設定
4. **cv_bridge使用**: OpenCV画像とROS2 Imageの相互変換
5. **複合データ通信**: 数値、文字列、画像の同時送信
6. **参考資料準拠**: 「Creating custom msg and srv files」の実装パターン

これにより、ROS 2におけるカスタムメッセージの作成から使用までの完全な流れを学習できます。
