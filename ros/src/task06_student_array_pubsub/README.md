# 課題6: カスタムメッセージを使えるようになる 2（動的配列）

## 概要
「Creating custom msg and srv files」チュートリアルに基づき、動的配列を利用したカスタムメッセージ`StudentArray`を使った学生情報配列通信を実装します。

**課題内容**: 学生情報を複数作成し、それらを1つにまとめて送受信できるメッセージを作る。動的配列を利用して、作成したメッセージをPublish・Subscriberで確認する。

## 機能

- **StudentArray Message**: 複数の学生情報を動的配列として格納するカスタムメッセージ
- **Student Array Publisher**: 複数の学生データを配列として生成・送信
- **Student Array Subscriber**: 学生情報配列を受信して表示

## パッケージ構造

```
student_array_pubsub/
├── student_array_pubsub/
│   ├── __init__.py
│   ├── student_array_publisher.py    # 学生情報配列Publisher
│   └── student_array_subscriber.py   # 学生情報配列Subscriber
├── resource/
│   └── student_array_pubsub
├── package.xml                       # task05_student_msgsに依存
├── setup.py                         # エントリーポイント設定
└── README.md
```

## 依存関係

### 前提パッケージ
- **task05_student_msgs**: カスタムメッセージ定義パッケージ（StudentInfo、StudentArray）
- **cv_bridge**: OpenCVとROS2の画像変換
- **opencv-python**: 画像処理ライブラリ

### カスタムメッセージ構造
```python
# task05_student_msgs/msg/StudentArray.msg
task05_student_msgs/StudentInfo[] students  # 動的配列：学生情報の可変長配列
int32 total_count                          # 配列の総数
std_msgs/Header header                     # メッセージのタイムスタンプ
```

**動的配列のポイント**: `[]`を使用することで可変長配列を定義し、実行時に配列サイズを決定できます。

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

**重要**: 依存関係の順序でビルドする必要があります

```bash
# ROS 2環境をセットアップ
source /opt/ros/humble/setup.bash
cd ~/ws

# 1. メッセージパッケージをビルド
colcon build --packages-select task05_student_msgs
source install/setup.bash

# 2. 配列Pub/Subパッケージをビルド
colcon build --packages-select student_array_pubsub
source install/setup.bash
```

### 3. インターフェース確認
```bash
# カスタムメッセージ構造を確認
ros2 interface show task05_student_msgs/msg/StudentArray

# 期待される出力:
# task05_student_msgs/StudentInfo[] students
#         int32 student_id
#         string name
#         sensor_msgs/Image image
#                 ... (詳細構造)
# int32 total_count
# std_msgs/Header header
#         builtin_interfaces/Time stamp
#                 int32 sec
#                 uint32 nanosec
#         string frame_id
```

## 実行方法

### ターミナル1: Publisher実行
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
python3 src/task06_student_array_pubsub/student_array_pubsub/student_array_publisher.py
```

**期待される出力**:
```
[INFO] [student_array_publisher]: Student Array Publisher node has been started.
[INFO] [student_array_publisher]: Publishing student array with 5 students
[INFO] [student_array_publisher]: Student 1: ID=20240001, Name="Tanaka Taro"
[INFO] [student_array_publisher]: Student 2: ID=20240002, Name="Sato Hanako"
[INFO] [student_array_publisher]: Student 3: ID=20240003, Name="Suzuki Ichiro"
[INFO] [student_array_publisher]: Student 4: ID=20240004, Name="Watanabe Yuki"
[INFO] [student_array_publisher]: Student 5: ID=20240005, Name="Ito Kazuko"
```

### ターミナル2: Subscriber実行
```bash
# 新しいターミナルでコンテナにアクセス
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
python3 src/task06_student_array_pubsub/student_array_pubsub/student_array_subscriber.py
```

**期待される出力**:
```
[INFO] [student_array_subscriber]: Student Array Subscriber node has been started.

=== Student Array Information ===
Total students: 5
Timestamp: 2024.01.15 12:34:56

Student #1:
  ID: 20240001
  Name: Tanaka Taro
  Image size: 300x200
  Image encoding: bgr8

Student #2:
  ID: 20240002
  Name: Sato Hanako
  Image size: 300x200
  Image encoding: bgr8

... (5人分の学生情報)
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

# 学生配列トピックの詳細
ros2 topic info /student_array

# メッセージ内容確認（一度だけ）
ros2 topic echo /student_array --once
```

## 実装のポイント

### Python コードでの動的配列使用

**Publisher側**:
```python
from task05_student_msgs.msg import StudentInfo, StudentArray
from std_msgs.msg import Header

# StudentArray メッセージ作成
array_msg = StudentArray()

# ヘッダー設定（タイムスタンプ付き）
array_msg.header = Header()
array_msg.header.stamp = self.get_clock().now().to_msg()
array_msg.header.frame_id = "student_array"

# 学生情報配列作成（動的配列）
students = []
for i in range(5):  # 5人の学生を作成
    student = StudentInfo()
    student.student_id = 20240001 + i
    student.name = student_names[i]
    student.image = self.create_student_image()
    students.append(student)

# 配列をメッセージに設定
array_msg.students = students
array_msg.total_count = len(students)

# 送信
self.publisher_.publish(array_msg)
```

**Subscriber側**:
```python
from task05_student_msgs.msg import StudentArray

def array_callback(self, msg):
    print(f"Total students: {msg.total_count}")

    # 動的配列をループで処理
    for i, student in enumerate(msg.students):
        print(f"Student #{i+1}:")
        print(f"  ID: {student.student_id}")
        print(f"  Name: {student.name}")
        # 画像データも処理可能
        cv_image = self.cv_bridge.imgmsg_to_cv2(student.image, "bgr8")
```

### package.xml での依存関係設定
```xml
<depend>task05_student_msgs</depend>
<depend>rclpy</depend>
<depend>sensor_msgs</depend>
<depend>std_msgs</depend>
<depend>cv_bridge</depend>
```

### setup.py でのエントリーポイント設定
```python
entry_points={
    'console_scripts': [
        'student_array_publisher = student_array_pubsub.student_array_publisher:main',
        'student_array_subscriber = student_array_pubsub.student_array_subscriber:main',
    ],
},
```

## 送信されるサンプルデータ

Publisherは以下の5人の学生データを1つの配列として送信します：
- ID: 20240001, Name: "Tanaka Taro"
- ID: 20240002, Name: "Sato Hanako"
- ID: 20240003, Name: "Suzuki Ichiro"
- ID: 20240004, Name: "Watanabe Yuki"
- ID: 20240005, Name: "Ito Kazuko"

各学生データには300x200のカラー画像とタイムスタンプが含まれます。

## 課題5との違い

| 項目 | 課題5 (StudentInfo) | 課題6 (StudentArray) |
|------|-------------------|---------------------|
| メッセージ型 | 単一学生情報 | 学生情報の動的配列 |
| 送信頻度 | 1人ずつ順次送信 | 5人まとめて一括送信 |
| データサイズ | 小（1人分） | 大（5人分） |
| タイムスタンプ | なし | Headerで管理 |
| 実装 | シンプル | 配列処理が必要 |

## トラブルシューティング

### メッセージパッケージが見つからない
```bash
# メッセージパッケージを先にビルド
cd ~/ws
colcon build --packages-select task05_student_msgs
source install/setup.bash

# その後配列Pub/Subパッケージをビルド
colcon build --packages-select student_array_pubsub
source install/setup.bash
```

### ImportError: No module named 'task05_student_msgs'
```bash
# 環境変数を確認
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

# メッセージパッケージがビルドされているか確認
ros2 pkg list | grep task05_student_msgs
ros2 interface list | grep StudentArray
```

### 大きなメッセージの送信で警告が出る場合
```bash
# ROS 2 DDS設定の確認（通常は問題なし）
echo $RMW_IMPLEMENTATION

# 必要に応じてQoS設定を調整（コード内で対応）
```

### 配列インデックスエラー
```python
# Pythonコードでの安全な配列アクセス
if len(msg.students) > 0:
    for i, student in enumerate(msg.students):
        # 処理
```

## 学習のポイント

1. **動的配列の定義**: `.msg`ファイルでの`[]`記法による可変長配列定義
2. **配列操作**: Pythonでのリスト操作とROS2メッセージの相互変換
3. **効率的なデータ転送**: 複数データを一括送信することによる通信効率向上
4. **ヘッダー付きメッセージ**: `std_msgs/Header`によるタイムスタンプ管理
5. **大容量データ処理**: 複数の画像を含む大きなメッセージの送受信
6. **参考資料準拠**: 「Creating custom msg and srv files」の動的配列パターン

これにより、ROS 2における動的配列を使った複雑なカスタムメッセージの作成から使用までの完全な流れを学習できます。
