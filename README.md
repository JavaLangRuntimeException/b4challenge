# B4 課題プロジェクト

このプロジェクトは、研究室で使用される**Linux (Ubuntu 22.04)** と **Docker** の基礎知識習得を目的とした課題集です。

## プロジェクト構成

```
b4/
├── linux/              # Linux (Ubuntu 22.04) 課題
│   ├── README.md       # Linux課題の詳細説明と実行手順
│   └── task3.sh        # シェルスクリプト課題
├── docker/             # Docker 課題
│   ├── README.md       # Docker課題の詳細説明と実行手順
│   ├── task3/          # Dockerfile作成課題
│   │   └── Dockerfile
│   ├── task4/          # Docker Compose課題
│   │   ├── Dockerfile
│   │   └── docker-compose.yml
│   └── task5/          # WebSocket通信課題
│       ├── server.py
│       ├── client.py
│       ├── Dockerfile.server
│       ├── Dockerfile.client
│       └── docker-compose.yml
└── README.md           # このファイル
```

## 課題概要

### Linux (Ubuntu 22.04) 課題

**目標**
- Linux (Ubuntu 22.04) の基礎知識習得
- Linux環境のセットアップ

**内容**
1. WSL2でUbuntu 22.04を実行
2. Linuxコマンドの学習（ディレクトリ・ファイル操作、パッケージ管理）
3. シェルスクリプトの作成

詳細は [`linux/README.md`](./linux/README.md) をご覧ください。

### Docker 課題

**目標**
- Dockerの基礎知識習得

**内容**
1. Dockerのインストール
2. コンテナの起動
3. Dockerfileを使ったイメージ作成
4. Docker Composeを使った複数コンテナ管理
5. コンテナ間WebSocket通信

詳細は [`docker/README.md`](./docker/README.md) をご覧ください。

## 使用方法

### Linux課題の実行

1. Linux課題ディレクトリに移動
```bash
cd linux
```

2. READMEを参照して課題を実行
```bash
cat README.md
```

3. シェルスクリプトの実行例
```bash
./task3.sh newfile.txt
```

### Docker課題の実行

1. Docker課題ディレクトリに移動
```bash
cd docker
```

2. 各タスクディレクトリで課題を実行

**課題3（Dockerfile作成）**
```bash
cd task3
docker build -t b4_docker .
docker run --rm -it b4_docker neofetch
```

**課題4（Docker Compose）**
```bash
cd task4
docker-compose up --build
```

**課題5（WebSocket通信）**
```bash
cd task5
docker-compose up --build
```

## 前提条件

### Linux課題
- Windows 10/11
- WSL2が有効化されていること
- Ubuntu 22.04がインストールされていること

### Docker課題
- Docker Desktop がインストールされていること
- Docker Compose が利用可能であること

## 注意事項

- WSL2では破壊的なコマンドがWindows環境に影響する可能性があります
- コマンドの意味を理解してから実行してください
- ポート番号の競合に注意してください（80番、8765番ポートを使用）
- コンテナを削除すると内部のデータは失われます

## 参考資料

### Linux関連
- [Microsoft WSL公式ドキュメント](https://learn.microsoft.com/ja-jp/windows/wsl/install)
- [Linux command入門](https://jellyware.jp/kurage/movidius/c04command_nano.html)

### Docker関連
- [Docker公式ドキュメント](https://docs.docker.jp/get-started/index.html)
- [Python WebSocket通信](https://www.raspberrypirulo.net/entry/websocket-server)

## ライセンス

このプロジェクトは教育目的で作成されています。
