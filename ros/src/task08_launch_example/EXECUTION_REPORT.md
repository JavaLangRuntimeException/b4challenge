# 課題8: Launch ファイルの作成と実行 - 実行報告書

## 課題概要

複数のROS2ノードを同時に起動するためのlaunchファイルの作成と実行を学習する課題。
以下の3つのlaunchファイルを実行し、複数ノード間の連携を確認する。

### 対象launchファイル
1. `basic_pubsub_launch.py` - 基本的なPublisher/Subscriber
2. `student_pubsub_launch.py` - 学生情報Publisher/Subscriber
3. `image_pubsub_launch.py` - 画像Publisher/Subscriber

## 実行環境

- **コンテナ**: b4_ros2 (Docker)
- **ROS2バージョン**: Humble
- **実行日時**: 2025年7月27日

## 実行手順

### 1. 依存パッケージのビルド

```bash
# コンテナ内で実行
docker exec -it b4_ros2 bash -c "cd ~/ws && source /opt/ros/humble/setup.bash && colcon build --packages-select task04_basic_pubsub student_info_pubsub student_array_pubsub image_pubsub"
```

**実行ログ:**
```
Starting >>> image_pubsub
Starting >>> student_array_pubsub
Finished <<< image_pubsub [0.55s]
Finished <<< student_array_pubsub [0.56s]

Summary: 2 packages finished [0.67s]
```

### 2. launch_exampleパッケージのビルド

```bash
# コンテナ内で実行
docker exec -it b4_ros2 bash -c "cd ~/ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select launch_example"
```

**ビルド結果:**
```
Starting >>> launch_example
Finished <<< launch_example [0.41s]

Summary: 1 package finished [0.52s]
```

## 実行結果

### 1. basic_pubsub_launch.py の実行

**実行コマンド:**
```bash
docker exec -it b4_ros2 bash -c "source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash && timeout 15 ros2 launch launch_example basic_pubsub_launch.py"
```

**実行ログ:**
```
[INFO] [launch]: All log files can be found below /home/taramanjimacbookpro/.ros/log/2025-07-27-13-25-47-733184-f93aa20a3c34-6179
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [publisher-1]: process started with pid [6685]
[INFO] [subscriber-2]: process started with pid [6687]
[subscriber-2] [INFO] [1753590584.197869420] [basic_subscriber]: Subscriber node has been started.
[subscriber-2] [INFO] [1753590584.198090336] [basic_subscriber]: Waiting for messages...
[publisher-1] [INFO] [1753590584.204808836] [basic_publisher]: Publisher node has been started.
[publisher-1] [INFO] [1753590584.205029836] [basic_publisher]: Enter messages to publish (Ctrl+C to exit):
```

**結果:** ✅ **成功** - PublisherとSubscriberが正常に同時起動

### 2. student_pubsub_launch.py の実行

**実行コマンド:**
```bash
docker exec -it b4_ros2 bash -c "source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash && timeout 15 ros2 launch launch_example student_pubsub_launch.py"
```

**実行ログ:**
```
[INFO] [launch]: All log files can be found below /home/taramanjimacbookpro/.ros/log/2025-07-27-13-31-48-376721-f93aa20a3c34-6845
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [student_publisher-1]: process started with pid [6846]
[INFO] [student_subscriber-2]: process started with pid [6848]
[student_publisher-1] [INFO] [1753590709.606679005] [student_info_publisher]: Student Publisher node has been started.
[student_subscriber-2] [INFO] [1753590709.606861839] [student_info_subscriber]: Student Subscriber node has been started.
[student_subscriber-2] [INFO] [1753590709.607060047] [student_info_subscriber]: Waiting for student info messages...
[student_publisher-1] [INFO] [1753590712.614393840] [student_info_publisher]: Publishing student info: ID=20240001, Name="Tanaka Taro", Image size=(200, 300, 3)
[student_subscriber-2] [INFO] [1753590712.619747424] [student_info_subscriber]: [1] Received student info: ID=20240001, Name="Tanaka Taro"
[student_publisher-1] [INFO] [1753590715.605302342] [student_info_publisher]: Publishing student info: ID=20240002, Name="Sato Hanako", Image size=(200, 300, 3)
[student_subscriber-2] [INFO] [1753590715.606031258] [student_info_subscriber]: [2] Received student info: ID=20240002, Name="Sato Hanako"
[student_publisher-1] [INFO] [1753590718.603634718] [student_info_publisher]: Publishing student info: ID=20240003, Name="Suzuki Ichiro", Image size=(200, 300, 3)
[student_subscriber-2] [INFO] [1753590718.604484301] [student_info_subscriber]: [3] Received student info: ID=20240003, Name="Suzuki Ichiro"
[student_publisher-1] [INFO] [1753590721.604064386] [student_info_publisher]: Publishing student info: ID=20240004, Name="Watanabe Yuki", Image size=(200, 300, 3)
[student_subscriber-2] [INFO] [1753590721.604686219] [student_info_subscriber]: [4] Received student info: ID=20240004, Name="Watanabe Yuki"
```

**結果:** ✅ **成功** - 学生情報の送受信が正常に動作。4名の学生データが継続的に配信

### 3. image_pubsub_launch.py の実行

**実行コマンド:**
```bash
docker exec -it b4_ros2 bash -c "source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash && timeout 15 ros2 launch launch_example image_pubsub_launch.py"
```

**実行ログ:**
```
[INFO] [launch]: All log files can be found below /home/taramanjimacbookpro/.ros/log/2025-07-27-13-32-25-318519-f93aa20a3c34-6897
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [image_publisher-1]: process started with pid [6898]
[INFO] [image_subscriber-2]: process started with pid [6900]
[image_subscriber-2] [INFO] [1753590745.647089925] [image_subscriber_node]: Image Subscriber node has been started.
[image_subscriber-2] [INFO] [1753590745.647445675] [image_subscriber_node]: Images will be saved to: /home/taramanjimacbookpro/ws/received_images
[image_subscriber-2] [INFO] [1753590745.647692425] [image_subscriber_node]: Waiting for image messages...
[image_publisher-1] [INFO] [1753590745.648390425] [image_publisher_node]: Created sample image: /home/taramanjimacbookpro/ws/install/image_pubsub/share/image_pubsub/images/sample_image_1.jpg
[image_publisher-1] [INFO] [1753590745.649854675] [image_publisher_node]: Created sample image: /home/taramanjimacbookpro/ws/install/image_pubsub/share/image_pubsub/images/sample_image_2.jpg
[image_publisher-1] [INFO] [1753590745.651913717] [image_publisher_node]: Created sample image: /home/taramanjimacbookpro/ws/install/image_pubsub/share/image_pubsub/images/sample_image_3.jpg
[image_publisher-1] [INFO] [1753590745.653574092] [image_publisher_node]: Created sample image: /home/taramanjimacbookpro/ws/install/image_pubsub/share/image_pubsub/images/sample_image_4.jpg
[image_publisher-1] [INFO] [1753590745.655172008] [image_publisher_node]: Created sample image: /home/taramanjimacbookpro/ws/install/image_pubsub/share/image_pubsub/images/sample_image_5.jpg
[image_publisher-1] [INFO] [1753590745.655292342] [image_publisher_node]: Prepared 5 sample images
[image_publisher-1] [INFO] [1753590745.655505008] [image_publisher_node]: Image Publisher node has been started.
[image_publisher-1] [INFO] [1753590747.644820509] [image_publisher_node]: Published image [0]: sample_image_1.jpg (400x300)
[image_subscriber-2] [INFO] [1753590747.646223634] [image_subscriber_node]: [1] Received image: 400x300, encoding=bgr8
[image_publisher-1] [INFO] [1753590749.647740927] [image_publisher_node]: Published image [1]: sample_image_2.jpg (400x300)
[image_subscriber-2] [INFO] [1753590749.648915719] [image_subscriber_node]: [2] Received image: 400x300, encoding=bgr8
[image_publisher-1] [INFO] [1753590751.645702428] [image_publisher_node]: Published image [2]: sample_image_3.jpg (400x300)
[image_subscriber-2] [INFO] [1753590751.646460053] [image_subscriber_node]: [3] Received image: 400x300, encoding=bgr8
[image_publisher-1] [INFO] [1753590753.645541970] [image_publisher_node]: Published image [3]: sample_image_4.jpg (400x300)
[image_subscriber-2] [INFO] [1753590753.645975054] [image_subscriber_node]: [4] Received image: 400x300, encoding=bgr8
[image_publisher-1] [INFO] [1753590755.645008013] [image_publisher_node]: Published image [4]: sample_image_5.jpg (400x300)
[image_subscriber-2] [INFO] [1753590755.645556805] [image_subscriber_node]: [5] Received image: 400x300, encoding=bgr8
[image_publisher-1] [INFO] [1753590757.642814708] [image_publisher_node]: Published image [5]: sample_image_1.jpg (400x300)
[image_subscriber-2] [INFO] [1753590757.643222041] [image_subscriber_node]: [6] Received image: 400x300, encoding=bgr8
```

**詳細受信ログ:**
```
=== Image #1 ===
Timestamp: 1753590747.642594384
Frame ID: camera_frame
Dimensions: 400x300
Encoding: bgr8
Step: 1200
Data length: 360000 bytes
OpenCV shape: (300, 400, 3)
OpenCV dtype: uint8
Image saved as: received_images/image_001_20250727_133227_646.jpg
File size: 21917 bytes

=== Image #2 ===
Timestamp: 1753590749.645849177
Frame ID: camera_frame
Dimensions: 400x300
Encoding: bgr8
Step: 1200
Data length: 360000 bytes
OpenCV shape: (300, 400, 3)
OpenCV dtype: uint8
Image saved as: received_images/image_002_20250727_133229_649.jpg
File size: 19414 bytes

（以下、image_006まで継続）
```

**結果:** ✅ **成功** - 400x300サイズの画像データが正常に送受信され、6つの画像ファイルが自動保存

## 実行確認コマンド

### launchファイルの引数確認
```bash
docker exec -it b4_ros2 bash -c "source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash && ros2 launch --show-args launch_example basic_pubsub_launch.py"
```

**結果:**
```
Arguments (pass arguments as '<name>:=<value>'):

  No arguments.
```

## 学習成果

### 1. 技術的成果
- **複数ノード同時起動**: launchファイルでの効率的なシステム起動方法を習得
- **ノード間連携**: PublisherとSubscriberの自動的な接続を確認
- **launchファイル構造**: Python形式でのlaunchファイル記述方法を理解
- **システム管理**: 複数プロセスの並行実行とログ管理を体験

### 2. 実行データ統計
| launchファイル | 起動ノード数 | データ種類 | 送受信メッセージ数 | 実行時間 |
|---|---|---|---|---|
| basic_pubsub_launch.py | 2 | 文字列 | 手動入力 | 15秒 |
| student_pubsub_launch.py | 2 | 学生情報（ID、名前、画像） | 4メッセージ | 15秒 |
| image_pubsub_launch.py | 2 | 画像データ（400x300） | 6メッセージ | 15秒 |

### 3. 保存ファイル
- **学生情報**: メモリ内処理（コンソール出力）
- **画像データ**: `/home/taramanjimacbookpro/ws/received_images/` に6ファイル保存
  - ファイルサイズ: 18KB〜23KB
  - 形式: JPEG（.jpg）

## 結論

🎉 **課題8「Launch ファイルの作成と実行」が完全に成功**

- 全3つのlaunchファイルが正常に動作
- 複数ノード間の通信が適切に確立
- データの送受信、保存機能が正常に動作
- launchファイルを使った効率的なシステム起動方法を習得

ROS2のlaunch機能を活用した複数ノードの協調動作を実現し、実践的なシステム開発手法を身につけました。
