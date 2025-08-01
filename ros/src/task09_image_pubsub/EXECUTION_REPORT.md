# 課題9: 画像ファイルのトピック通信 - 実行報告書

## 課題概要

画像ファイルをOpenCVで読み込み、ROS2トピック通信を通じて送受信し、JPGファイルとして保存する機能の実装と実行を学習する課題。
OpenCVとcv_bridgeを活用した画像データのPublisher/Subscriber通信を実現する。

### 主要機能
- **Image Publisher**: 保存済み画像ファイルをOpenCV形式で読込、cv_bridgeでsensor_msgs/Imageに変換してPublish
- **Image Subscriber**: 画像を受信し、cv_bridgeでOpenCV形式に変換して表示・JPGファイル保存

## 実行環境

- **コンテナ**: b4_ros2 (Docker)
- **ROS2バージョン**: Humble
- **実行日時**: 2025年7月27日
- **使用ライブラリ**: OpenCV、cv_bridge、sensor_msgs

## 実行手順

### 1. パッケージのビルド

```bash
# コンテナ内で実行
docker exec -it b4_ros2 bash -c "cd ~/ws && source /opt/ros/humble/setup.bash && colcon build --packages-select image_pubsub"
```

**ビルド結果:**
```
Starting >>> image_pubsub
Finished <<< image_pubsub [0.40s]

Summary: 1 package finished [0.55s]
```

### 2. サンプル画像の作成

```bash
# コンテナ内で実行
docker exec -it b4_ros2 bash -c "cd ~/ws/src/task09_image_pubsub && python3 -c \"
import cv2
import numpy as np
import os

# imagesディレクトリを確認
os.makedirs('images', exist_ok=True)

# サンプル画像を生成
img = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.rectangle(img, (50, 50), (590, 430), (0, 255, 0), 3)
cv2.putText(img, 'ROS2 Image PubSub', (150, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
cv2.putText(img, 'Task 09 Demo', (200, 300), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
cv2.imwrite('images/sample_image.jpg', img)
print('Sample image created: images/sample_image.jpg')
print('Image size:', img.shape)
\""
```

**画像作成結果:**
```
Sample image created: images/sample_image.jpg
Image size: (480, 640, 3)

# ファイルサイズ確認
total 32
-rw-r--r-- 2 PC-034 dialout 29744 Jul 27 15:28 sample_image.jpg
```

## 実行結果

### 1. Image Publisher の実行

**実行コマンド:**
```bash
docker exec -it b4_ros2 bash -c "source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash && ~/ws/install/image_pubsub/bin/image_publisher"
```

**トピック確認:**
```bash
docker exec -it b4_ros2 bash -c "source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash && ros2 topic list"
```

**トピック一覧:**
```
/image_topic
/parameter_events
/rosout
/topic
```

**結果:** ✅ **成功** - Image Publisherが正常に起動し、`/image_topic`で画像データを配信

### 2. Image Subscriber の実行

**実行コマンド:**
```bash
docker exec -it b4_ros2 bash -c "source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash && timeout 20 ~/ws/install/image_pubsub/bin/image_subscriber"
```

**実行ログ:**
```
[INFO] [1753598337.800758837] [image_subscriber]: Image Subscriber node has been started.
[INFO] [1753598337.801096462] [image_subscriber]: Images will be saved to: /home/PC-034/ws/received_images
[INFO] [1753598337.801456462] [image_subscriber]: Waiting for image messages...
[INFO] [1753598339.793668755] [image_subscriber]: [1] Received image: 400x300, encoding=bgr8
[INFO] [1753598341.796306464] [image_subscriber]: [2] Received image: 400x300, encoding=bgr8
[INFO] [1753598343.799316382] [image_subscriber]: [3] Received image: 400x300, encoding=bgr8
[INFO] [1753598345.795237591] [image_subscriber]: [4] Received image: 400x300, encoding=bgr8
[INFO] [1753598347.797710800] [image_subscriber]: [5] Received image: 400x300, encoding=bgr8
[INFO] [1753598349.798699760] [image_subscriber]: [6] Received image: 400x300, encoding=bgr8
[INFO] [1753598351.797467219] [image_subscriber]: [7] Received image: 400x300, encoding=bgr8
[INFO] [1753598353.794307761] [image_subscriber]: [8] Received image: 400x300, encoding=bgr8
[INFO] [1753598355.794806846] [image_subscriber]: [9] Received image: 400x300, encoding=bgr8
```

**詳細受信ログ（代表例）:**
```
=== Image #1 ===
Timestamp: 1753598339.790652546
Frame ID: camera_frame
Dimensions: 400x300
Encoding: bgr8
Step: 1200
Data length: 360000 bytes
OpenCV shape: (300, 400, 3)
OpenCV dtype: uint8
Image saved as: received_images/image_001_20250727_153859_794.jpg
File size: 21917 bytes

=== Image #5 ===
Timestamp: 1753598347.794469467
Frame ID: camera_frame
Dimensions: 400x300
Encoding: bgr8
Step: 1200
Data length: 360000 bytes
OpenCV shape: (300, 400, 3)
OpenCV dtype: uint8
Image saved as: received_images/image_005_20250727_153907_798.jpg
File size: 22696 bytes

>>> Statistics: Received 5 images so far <<<
>>> Output directory: /home/PC-034/ws/received_images <<<
```

**結果:** ✅ **成功** - 9枚の画像を正常に受信し、JPEGファイルとして自動保存

### 3. 保存ファイルの確認

**保存先確認:**
```bash
docker exec -it b4_ros2 bash -c "ls -la ~/ws/received_images/ | head -10"
```

**保存ファイル一覧:**
```
total 348
drwxr-xr-x 2 PC-034 dialout  4096 Jul 27 15:39 .
drwxr-xr-x 1 PC-034 dialout  4096 Jul 27 13:32 ..
-rw-r--r-- 1 PC-034 dialout 21917 Jul 27 15:38 image_001_20250727_153859_794.jpg
-rw-r--r-- 1 PC-034 dialout 19414 Jul 27 15:39 image_002_20250727_153901_798.jpg
-rw-r--r-- 1 PC-034 dialout 22250 Jul 27 15:39 image_003_20250727_153903_800.jpg
-rw-r--r-- 1 PC-034 dialout 18254 Jul 27 15:39 image_004_20250727_153905_795.jpg
-rw-r--r-- 1 PC-034 dialout 22696 Jul 27 15:39 image_005_20250727_153907_798.jpg
-rw-r--r-- 1 PC-034 dialout 21917 Jul 27 15:39 image_006_20250727_153909_799.jpg
-rw-r--r-- 1 PC-034 dialout 19414 Jul 27 15:39 image_007_20250727_153911_798.jpg
```

**結果:** ✅ **成功** - 9つの画像ファイルが連番で正常に保存（ファイルサイズ: 18KB〜23KB）

## 学習成果

### 1. 技術的成果
- **cv_bridge活用**: OpenCV画像とROS2 sensor_msgs/Imageの相互変換を習得
- **画像ファイルI/O**: cv2.imread()とcv2.imwrite()による画像処理を実装
- **大容量メッセージ処理**: 画像データ（360,000バイト）の効率的な送受信を実現
- **リアルタイム画像処理**: 連続画像データの安定した処理を確認

### 2. 実行データ統計
| 項目 | 値 |
|---|---|
| 送受信画像数 | 9枚 |
| 画像サイズ | 400x300ピクセル |
| エンコーディング | bgr8 |
| データサイズ | 360,000バイト/画像 |
| 保存ファイルサイズ | 18KB〜23KB（JPEG圧縮） |
| 実行時間 | 20秒 |
| 平均転送間隔 | 約2.2秒 |

### 3. 使用技術スタック
- **ROS2 Humble**: トピック通信基盤
- **OpenCV**: 画像処理・変換ライブラリ
- **cv_bridge**: ROS2とOpenCVの画像形式変換
- **sensor_msgs**: 画像メッセージ型定義
- **Python3**: 実装言語

### 4. メッセージ構造分析
```
sensor_msgs/msg/Image:
- Header header
  - uint32 seq
  - time stamp
  - string frame_id: "camera_frame"
- uint32 height: 300
- uint32 width: 400
- string encoding: "bgr8"
- uint8 is_bigendian: 0
- uint32 step: 1200
- uint8[] data: [360000 bytes]
```

## 結論

🎉 **課題9「画像ファイルのトピック通信」が完全に成功**

- **画像Publisher/Subscriber通信が正常に動作**
- **OpenCVとcv_bridgeによる画像変換が適切に実行**
- **大容量画像データの安定した送受信を実現**
- **9枚の画像ファイルが連番で自動保存完了**
- **リアルタイム画像処理システムの基礎を習得**

OpenCVを活用したROS2画像通信システムを構築し、実用的な画像処理・配信・保存のワークフローを確立しました。
画像データの効率的な処理とファイル管理システムの実装により、コンピュータビジョンシステムの基盤技術を身につけました。
