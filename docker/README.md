# Docker 課題

## 目標
- 「ROS 2」課題などで利用するDockerの基礎知識習得

## 課題内容

### 1. Docker をインストールする

Docker Desktop をインストールし、Dockerを使用できる環境を構築します。

#### インストール方法
以下の記事を参考にしてください：
- [Docker公式ドキュメント](https://docs.docker.jp/get-started/index.html)
- [Qiita記事](https://qiita.com/hoshimado/items/51c99ccaee3d4222d99d#docker%E7%92%B0%E5%A2%83%E3%81%AE%E5%8B%95%E4%BD%9C%E7%A2%BA%E8%AA%8D)

#### 理解すべき内容

- **Docker とは何か？そのメリットは何か？**
  - Docker：アプリケーションをコンテナという軽量な仮想環境で実行するプラットフォーム
  - メリット：
    - 環境の一貫性（どこでも同じ環境で実行可能）
    - 軽量（仮想マシンより高速起動・少ないリソース消費）
    - ポータビリティ（異なるOS間での移植性）
    - スケーラビリティ（簡単にスケールアップ・ダウン可能）

- **従来の仮想マシンとの違いは何か？**
  - 仮想マシン：ハードウェアレベルで仮想化、各VMに完全なOSが必要
  - Docker：OSレベルで仮想化、ホストOSのカーネルを共有
  - Dockerの方が軽量で高速起動が可能

### 2. コンテナを起動する

Docker Runコマンドを使用して、コンテナを起動します。

以下の条件でコンテナを起動し、"http://localhost:80/" にアクセスします。

**起動するコンテナの条件**
- イメージ: `docker/getting-started`
- ポート: コンテナ内の80番ポートにホストマシンの80番ポートを割り当てる
- モード: デタッチド・モード（バックグラウンド）で実行

#### 実行手順

1. コンテナを起動
```bash
docker run -d -p 80:80 docker/getting-started
```

2. ブラウザで確認
`http://localhost:80/` にアクセスして、Docker getting-startedページが表示されることを確認

3. 実行中のコンテナを確認
```bash
docker ps
```

#### 理解すべき内容

- **コンテナとは何か？**
  - Dockerイメージから作成された実行可能なインスタンス
  - アプリケーションとその依存関係を含む軽量な実行環境

- **イメージとは何か？**
  - コンテナを作成するためのテンプレート
  - アプリケーション、ライブラリ、設定ファイルなどが含まれる

- **実行中のコンテナを停止するにはどうすればよいか？**
```bash
docker ps              # 実行中のコンテナ一覧を表示
docker stop <コンテナID>  # 指定したコンテナを停止
docker stop $(docker ps -q)  # 全ての実行中コンテナを停止
```

- **コンテナ・イメージを削除するにはどうすればよいか？**
```bash
# コンテナの削除
docker rm <コンテナID>        # 停止したコンテナを削除
docker rm -f <コンテナID>     # 実行中のコンテナを強制削除
docker container prune       # 停止中の全コンテナを削除

# イメージの削除
docker rmi <イメージ名>        # イメージを削除
docker image prune          # 未使用イメージを削除
docker system prune         # 未使用のコンテナ、ネットワーク、イメージを一括削除
```

### 3. Dockerfile を使ってイメージを作成する

Dockerfileを使ってイメージを作成します。Dockerfileは、どのベースイメージを使用するか、どのようなアプリケーションをインストールするか、どのコマンドを実行するかなど、イメージ作成の手順を一行ずつ記述することで、新たなイメージを作成することができます。

「Linux (Ubuntu 22.04)」課題で学んだ知識を活かし、Ubuntu 22.04に"neofetch"をインストールするDockerfileおよびイメージを作成します。

**作成するイメージの条件**
- 作成するイメージの名前: `b4_docker`
- ベースイメージ: `ubuntu:22.04`
- インストールするアプリケーション: `neofetch`

#### 実行手順

1. task3ディレクトリに移動
```bash
cd docker/task3
```

2. イメージをビルド
```bash
docker build -t b4_docker .
```

3. コンテナを起動してneofetchを実行
```bash
docker run --rm -it b4_docker neofetch
```

neofetchの出力から、Ubuntu 22.04が動作していることを確認します。

### 4. Docker Compose を使う

Docker Composeを使って、複数のコンテナを一括で起動します。Docker Composeは、複数のDockerコンテナを1つのyamlファイルで設定することができます。

Docker Composeを使って同時に2つのコンテナを起動し、一方のコンテナからもう一方のコンテナに対してpingを飛ばします。

**"web" コンテナの条件**
- イメージ: `docker/getting-started`
- ポート: コンテナ内の80番ポートにホストマシンの80番ポートを割り当てる

**"app" コンテナの条件**
- イメージ: Dockerfileで定義（下記の条件でイメージを作成）
- コマンド: "web"コンテナに向けpingを飛ばす

**作成するイメージの条件**
- 作成するイメージの名前: `b4_compose`
- ベースイメージ: `ubuntu:22.04`
- インストールするアプリケーション: `iputils-ping`

#### 実行手順

1. task4ディレクトリに移動
```bash
cd docker/task4
```

2. Docker Composeでコンテナを起動
```bash
docker-compose up --build
```

3. ping応答が返ってくることを確認

4. コンテナを停止
```bash
docker-compose down
```

### 5. コンテナ間で WebSocket 通信を行う

ここまでで学習したDockerの知識を活かして、WebSocketで通信を行うプログラムを作成し、2つのコンテナ間での通信を行います。プログラムはPythonで作成し、Docker Composeによりサーバとクライアントの2つのコンテナを一括で起動できるようにします。

コンテナの削除などによりソースコードが失われないよう、注意してください。

余力があれば、通信内容とそのタイムスタンプをファイルに書き込み、コンテナを削除してもそのファイルが残るようにしてみると良いでしょう。

#### 実行手順

1. task5ディレクトリに移動
```bash
cd docker/task5
```

2. Docker Composeでコンテナを起動
```bash
docker-compose up --build
```

3. WebSocket通信が正常に行われることを確認

4. コンテナを停止
```bash
docker-compose down
```

#### 参考資料

- [Python を使った WebSocket 通信](https://www.raspberrypirulo.net/entry/websocket-server)
- [WebSocketクライアント](https://www.raspberrypirulo.net/entry/websocket-client)
- [ホストマシンのファイルをコンテナにコピーする](https://qiita.com/yyy752/items/627189dfbd410463325d)
- [コンテナのデータの永続化](https://zenn.dev/fire_arlo/articles/docker-volume-vs-bind-mount)

## よく使うDockerコマンド

### 基本的なコマンド

```bash
# イメージの一覧表示
docker images

# コンテナの一覧表示
docker ps        # 実行中のコンテナ
docker ps -a     # 全てのコンテナ

# コンテナの実行
docker run <イメージ名>
docker run -it <イメージ名> bash    # インタラクティブモードでbashを実行
docker run -d <イメージ名>          # デタッチドモード（バックグラウンド）で実行

# コンテナの停止・削除
docker stop <コンテナID>
docker rm <コンテナID>

# イメージの削除
docker rmi <イメージ名>

# ログの確認
docker logs <コンテナID>

# コンテナ内でコマンド実行
docker exec -it <コンテナID> bash
```

### Docker Composeコマンド

```bash
# コンテナの起動（ビルド込み）
docker-compose up --build

# バックグラウンドで起動
docker-compose up -d

# コンテナの停止・削除
docker-compose down

# ログの確認
docker-compose logs

# 特定のサービスのログ確認
docker-compose logs <サービス名>
```

## 注意事項

- コンテナを削除すると、そのコンテナ内で行った変更は失われます
- 重要なデータは永続化（Volume）を使用して保存してください
- ポート番号の競合に注意してください（既に使用されているポートは指定できません）
