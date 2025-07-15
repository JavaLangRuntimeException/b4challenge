#!/bin/bash

# シェルスクリプト課題3
# 実行方法: ./task3.sh <新しいファイル名>
# 例: ./task3.sh newfile.txt

# 引数チェック
if [ $# -eq 0 ]; then
    echo "使用方法: $0 <新しいファイル名>"
    echo "例: $0 newfile.txt"
    exit 1
fi

# 新しいファイル名を引数から取得
NEW_FILENAME="$1"

# 1. "----start----" とターミナルに表示
echo "----start----"

# 2. ホームディレクトリ直下に"task3"ディレクトリを作成
echo "task3ディレクトリを作成中..."
mkdir -p ~/task3

# 3. 課題2-aで作成したテキストファイルを"task3"にコピー
echo "hello.txtをtask3ディレクトリにコピー中..."
if [ -f ~/task2-a/hello.txt ]; then
    cp ~/task2-a/hello.txt ~/task3/
    echo "ファイルのコピーが完了しました"
else
    echo "警告: ~/task2-a/hello.txt が見つかりません"
    echo "課題2-aを先に実行してください"
    exit 1
fi

# 4. コピーしたテキストファイルの名前をシェルスクリプトに引数で与えた名前に変更
echo "ファイル名を ${NEW_FILENAME} に変更中..."
mv ~/task3/hello.txt ~/task3/"${NEW_FILENAME}"

# 5. コピーしたテキストファイルの内容をターミナルに表示（catコマンドを使う）
echo "ファイルの内容を表示します:"
cat ~/task3/"${NEW_FILENAME}"

# 6. "----end----" とターミナルに表示
echo "----end----"

echo ""
echo "スクリプトの実行が完了しました。"
echo "作成されたファイル: ~/task3/${NEW_FILENAME}"
