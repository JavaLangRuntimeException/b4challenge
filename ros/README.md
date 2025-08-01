# B4課題用 ROS 2 Dockerコンテナ環境
ROS 2課題の環境構築に使う。
本課題およびこのDockerコンテナ環境はmacOSでの動作を想定している。

# 事前準備
## 必要なソフトウェア
1. **Docker Desktop for Mac**: [公式サイト](https://www.docker.com/products/docker-desktop/)からダウンロードしてインストール
2. **XQuartz**: GUI表示のために必要
   ```bash
   brew install --cask xquartz
   ```
   または[公式サイト](https://www.xquartz.org/)からダウンロード

## XQuartzの設定
XQuartzをインストール後、以下の設定を行う：
1. XQuartzを起動
2. メニューバーから「XQuartz」→「環境設定」を選択
3. 「セキュリティ」タブで「ネットワーククライアントからの接続を許可」にチェック
4. XQuartzを再起動

# 使い方
## リポジトリのクローン
以下のコマンドを使って、このリポジトリをクローンする：
```bash
git clone --recursive https://github.com/RAIT-09/b4_ros2.git
cd b4_ros2
```

## .envファイルの確認
リポジトリに含まれる`.env`ファイルを確認し、必要に応じて編集する：
```bash
cat .env
```

## Dev Containersの利用
本リポジトリは、Visual Studio Code（以下 VS Code）のDev Containers機能を設定済である。

Dev Containersとは、VS CodeにおいてDockerコンテナを用いた開発環境を容易に構築・共有するための仕組みである。
VS Codeに「Dev Containers」拡張機能をインストールした上で、「コンテナーで再度開く（Reopen in Container）」を選択することで、指定されたDocker環境が自動的に立ち上がり、統一された開発環境が利用可能となる。

## コマンドラインでのコンテナ操作
### XQuartzの起動とX11転送の設定
```bash
# XQuartzを起動
open -a XQuartz

# X11転送を許可
xhost +localhost
```

### コンテナの起動
以下のコマンドを実行して、ROS 2コンテナを起動する：
```bash
docker compose up -d
```

### コンテナ内でコマンドを実行する
以下のコマンドを実行して、ROS 2コンテナのBashを使用する：
```bash
docker exec -it b4_ros2 bash
```

# GUI表示の確認
コンテナの起動後にRviz2を実行して、GUI表示が機能することを確認する：

```bash
# コンテナ内で実行
source /opt/ros/humble/setup.bash
rviz2
```

# ターミナルの多重化
tmuxを導入しているので、それを使用するとよい。

# 課題の実行方法

このリポジトリには、ROS2の学習用課題が含まれています。以下の順序で課題を実行することを推奨します。

## 課題実行の基本手順

### 1. 環境の起動
```bash
# リポジトリのルートディレクトリで実行
./start_macos.sh
```

このスクリプトが自動的に以下を実行します：
- XQuartzの起動確認
- Docker Desktopの起動確認  
- .envファイルの生成（UID/GID設定）
- Dockerコンテナの起動

### 2. コンテナへのアクセス
```bash
# 新しいターミナルウィンドウを開いて実行
docker exec -it b4_ros2 bash
```

### 3. ROS2環境のセットアップ（コンテナ内）
```bash
# ROS2環境をセットアップ
source /opt/ros/humble/setup.bash

# ワークスペースをビルド（初回のみ）
cd ~/ws
colcon build

# ワークスペースの環境をセットアップ
source install/setup.bash
```

## 課題の実行順序

各課題は以下の順序で実行することを推奨します：

### Phase 1: 基礎理解
1. **task01_02_turtlesim**: ROS2ノードとトピック通信の基礎
   - GUI表示の確認
   - turtlesimによる基本操作

2. **task03_package_creation**: パッケージ作成とビルド
   - ROS2パッケージの構造理解
   - colcon buildの使用方法

### Phase 2: 通信の実装
3. **task04_basic_pubsub**: 基本的なPublisher/Subscriber
   - 文字列メッセージの送受信
   - ノード間通信の基礎

4. **task05_student_msgs**: カスタムメッセージの作成
   - 独自メッセージ型の定義
   - メッセージパッケージのビルド

5. **task05_student_info_pubsub**: カスタムメッセージの使用
   - 学生情報メッセージの送受信
   - カスタムメッセージの実装

6. **task06_student_array_pubsub**: 配列メッセージの処理
   - 複数学生情報の送受信
   - 配列データの処理

### Phase 3: 高度な機能
7. **task07_bag**: データの記録と再生
   - rosbagを使用したデータ記録
   - 記録データの再生と解析

8. **task08_launch_example**: 起動ファイルの作成
   - launch fileによる複数ノードの管理
   - パラメータ設定

9. **task09_image_pubsub**: 画像データの処理
   - 画像メッセージの送受信
   - OpenCVとの連携

### Phase 4: システム統合
10. **task11_multi_container**: マルチマシン通信
    - 複数コンテナ間での通信
    - 分散システムの理解

## 各課題の詳細

各課題のディレクトリには詳細なREADME.mdファイルが含まれています：

```
src/
├── task01_02_turtlesim/README.md      # turtlesimとトピック通信
├── task03_package_creation/README.md  # パッケージ作成
├── task04_basic_pubsub/README.md      # 基本的な通信
├── task05_student_msgs/README.md      # カスタムメッセージ
├── task05_student_info_pubsub/README.md # 学生情報通信
├── task06_student_array_pubsub/README.md # 配列データ通信
├── task07_bag/README.md               # データ記録・再生
├── task08_launch_example/README.md    # 起動ファイル
├── task09_image_pubsub/README.md      # 画像処理
└── task11_multi_container/README.md   # マルチマシン通信
```

## 実行時の注意点

- **初回実行**: 必ず`task03_package_creation`で環境をビルドしてから他の課題を実行してください
- **GUI表示**: task01_02_turtlesimとtask09_image_pubsubではXQuartzが必要です
- **複数ターミナル**: 多くの課題では複数のターミナルウィンドウが必要です（tmuxの使用を推奨）
- **依存関係**: task05_student_msgsはtask05/06の学生情報関連課題の前提条件です

## 学習の進め方

1. 各課題のREADME.mdを読んで学習目標を理解
2. 実行手順に従ってコードを実行
3. 動作を確認し、コードの内容を理解
4. 必要に応じてコードを改良・拡張

# トラブルシューティング
## GUI表示がされない場合
1. XQuartzが起動していることを確認
2. `xhost +localhost`を実行していることを確認
3. 必要に応じてDockerコンテナを再起動

## 権限エラーが発生する場合
.envファイルのUIDとGIDを現在のユーザーに合わせて修正：
```bash
echo "UID=$(id -u)" >> .env
echo "GID=$(id -g)" >> .env
```

# 便利なスクリプト

## クイックスタート用のエイリアス設定

毎回長いコマンドを入力することを避けるため、以下のエイリアスを設定することをお勧めします：

```bash
# ~/.zshrcまたは~/.bashrcに追加
alias ros2_start="cd ~/git/b4_ros2 && ./start_macos.sh"
alias ros2_enter="docker exec -it b4_ros2 bash"
alias ros2_setup="source /opt/ros/humble/setup.bash && cd ~/ws && source install/setup.bash"
```

設定後は以下のように使用できます：
```bash
# 1. 環境起動
ros2_start

# 2. 新しいターミナルでコンテナアクセス
ros2_enter

# 3. コンテナ内でROS2環境セットアップ
ros2_setup
```

## 一括実行確認スクリプト

全ての課題が正常に動作することを確認するための簡単なテストスクリプトです：

```bash
# コンテナ内で実行するテストスクリプト例
#!/bin/bash
echo "=== ROS2課題環境テスト ==="

# 基本環境の確認
echo "1. ROS2環境確認..."
source /opt/ros/humble/setup.bash
ros2 --version

# ワークスペースのビルド確認
echo "2. ワークスペースビルド確認..."
cd ~/ws
colcon build --packages-select task05_student_msgs
source install/setup.bash

# 基本的なノード起動テスト
echo "3. 基本ノードテスト..."
timeout 5s ros2 run demo_nodes_cpp talker &
sleep 2
timeout 3s ros2 run demo_nodes_cpp listener
wait

echo "=== テスト完了 ==="
```

このテストスクリプトをコンテナ内で実行することで、基本的な環境が正常に動作することを確認できます。
