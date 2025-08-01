# 課題9: 画像ファイルのトピック通信

このパッケージは、画像ファイルをOpenCVで読み込み、ROS2トピック通信を通じて送受信し、JPGファイルとして保存する機能を実装します。

## 機能

- **Image Publisher**: 保存済み画像ファイルをOpenCV形式で読込、cv_bridgeでsensor_msgs/Imageに変換してPublish
- **Image Subscriber**: 画像を受信し、cv_bridgeでOpenCV形式に変換して表示・JPGファイル保存

## ファイル構成

```
image_pubsub/
├── image_pubsub/
│   ├── __init__.py
│   ├── image_publisher.py    # 画像Publisher
│   └── image_subscriber.py   # 画像Subscriber
├── images/                   # サンプル画像ディレクトリ
├── resource/
│   └── image_pubsub
├── package.xml
├── setup.py
└── README.md
```

## 前提条件

まず、Dockerコンテナにアクセスしてください：

```bash
# macOSホスト上で実行
cd /path/to/b4_ros2
./start_macos.sh

# コンテナにアクセス
docker exec -it b4_ros2 bash
```

## 使用方法

### 1. 画像ファイルの準備

```bash
# コンテナ内で実行
# サンプル画像を作成（OpenCVを使用）
cd ~/ws/src/task09_image_pubsub
python3 -c "
import cv2
import numpy as np
import os

# imagesディレクトリを作成
os.makedirs('images', exist_ok=True)

# サンプル画像を生成
img = np.zeros((480, 640, 3), dtype=np.uint8)
cv2.rectangle(img, (50, 50), (590, 430), (0, 255, 0), 3)
cv2.putText(img, 'ROS2 Sample Image', (200, 250), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
cv2.imwrite('images/sample_image.jpg', img)
print('Sample image created: images/sample_image.jpg')
"
```

### 2. ビルド

```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
cd ~/ws

# パッケージをビルド（画像ファイルの問題は修正済み）
colcon build --packages-select image_pubsub
source install/setup.bash
```

### 3. 実行

#### ターミナル1: Publisher実行
```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 run image_pubsub image_publisher
```

出力例：
```
[INFO] [1234567890.123456789] [image_publisher]: Image Publisher node has been started.
[INFO] [1234567890.123456789] [image_publisher]: Publishing image: sample_image.jpg, size: (640, 480)
[INFO] [1234567890.123456789] [image_publisher]: Image encoding: bgr8
```

#### ターミナル2: Subscriber実行
```bash
# 新しいターミナルでコンテナにアクセス
docker exec -it b4_ros2 bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
ros2 run image_pubsub image_subscriber
```

出力例：
```
[INFO] [1234567890.123456789] [image_subscriber]: Image Subscriber node has been started.
[INFO] [1234567890.123456789] [image_subscriber]: Received image: 640x480, encoding: bgr8
[INFO] [1234567890.123456789] [image_subscriber]: Image saved as: received_image_1.jpg
```

### 4. 結果確認

```bash
# コンテナ内で実行
# 保存された画像ファイルを確認
ls -la *.jpg

# 画像ファイルの詳細情報
file received_image_*.jpg
```

## 学習ポイント

1. **cv_bridge使用**: OpenCV画像とROS2 sensor_msgs/Imageの相互変換
2. **画像ファイルI/O**: cv2.imread()とcv2.imwrite()の使用
3. **画像エンコーディング**: bgr8, rgb8, mono8等の形式理解
4. **メッセージサイズ**: 画像データの大容量メッセージ処理
5. **リアルタイム画像処理**: 連続画像データの効率的な処理

## 依存関係

- `cv_bridge`: OpenCVとROS2の画像変換ライブラリ
- `opencv-python`: Python OpenCVライブラリ
- `sensor_msgs`: 画像メッセージ定義

## トピック構造

```bash
# コンテナ内で実行
# 画像トピックの詳細確認
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

# トピック情報
ros2 topic info /image_topic

# メッセージ構造確認
ros2 interface show sensor_msgs/msg/Image
```

## 高度な使用例

### リアルタイム画像処理
```bash
# コンテナ内で実行
# 複数画像の連続配信をテスト
# （画像ファイルを複数用意して循環配信）
```

### 画像圧縮
```bash
# コンテナ内で実行
# CompressedImageメッセージでの送信
ros2 topic echo /image_topic/compressed
```

## トラブルシューティング

### cv_bridgeが見つからない
```bash
# コンテナ内で実行
# OpenCVライブラリの確認
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"
python3 -c "from cv_bridge import CvBridge; print('cv_bridge imported successfully')"
```

### 画像ファイルが見つからない
```bash
# コンテナ内で実行
# 画像ディレクトリの確認
ls -la ~/ws/src/task09_image_pubsub/images/

# 画像ファイルの再作成
cd ~/ws/src/task09_image_pubsub
python3 -c "import cv2; import numpy as np; cv2.imwrite('images/test.jpg', np.zeros((100,100,3), np.uint8))"
```

### メッセージサイズエラー
```bash
# コンテナ内で実行
# ROS2の設定確認
ros2 param list

# より小さな画像でテスト
python3 -c "
import cv2
import numpy as np
img = np.zeros((240, 320, 3), dtype=np.uint8)
cv2.imwrite('images/small_image.jpg', img)
"
```

### コンテナアクセスの問題
```bash
# macOSホスト上で確認
docker ps | grep b4_ros2

# 起動していない場合
cd /path/to/b4_ros2
./start_macos.sh
```

## 応用課題

1. ウェブカメラからの画像をリアルタイムでPublish
2. 画像に対してOpenCVフィルタを適用してPublish
3. 複数の画像形式（PNG、JPEG、BMP）に対応
4. 画像のメタデータ（撮影時刻、サイズ等）を付加して送信
