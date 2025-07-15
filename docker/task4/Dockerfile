# Ubuntu 22.04をベースイメージとして使用
FROM ubuntu:22.04

# 非対話的インストールを設定
ENV DEBIAN_FRONTEND=noninteractive

# パッケージリストを更新し、iputils-pingをインストール
RUN apt-get update && \
    apt-get install -y iputils-ping && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# デフォルトコマンドを設定（webコンテナに向けてpingを実行）
CMD ["ping", "web"]
