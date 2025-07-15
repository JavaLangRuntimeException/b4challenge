if [ $# -eq 0 ]; then
    echo "使用方法: $0 <新しいファイル名>"
    echo "例: $0 newfile.txt"
    exit 1
fi

NEW_FILENAME="$1"

echo "----start----"

mkdir -p ~/task3

if [ -f ~/task2-a/hello.txt ]; then
    cp ~/task2-a/hello.txt ~/task3/
    echo "ファイルのコピーが完了しました"
else
    echo "警告: ~/task2-a/hello.txt が見つかりません"
    echo "課題2-aを先に実行してください"
    exit 1
fi

echo "ファイル名を ${NEW_FILENAME} に変更中..."
mv ~/task3/hello.txt ~/task3/"${NEW_FILENAME}"

echo "ファイルの内容を表示します:"
cat ~/task3/"${NEW_FILENAME}"

echo "----end----"

echo "作成されたファイル: ~/task3/${NEW_FILENAME}"
