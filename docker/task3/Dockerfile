# Ubuntu 22.04をベースイメージとして使用
FROM ubuntu:22.04

# 非対話的インストールを設定（タイムゾーン選択などを回避）
ENV DEBIAN_FRONTEND=noninteractive

# パッケージリストを更新し、neofetchをインストール
RUN apt-get update && \
    apt-get install -y neofetch && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# デフォルトコマンドをbashに設定
CMD ["/bin/bash"]
