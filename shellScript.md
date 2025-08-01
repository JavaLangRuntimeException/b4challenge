開発者の皆さん...UNIX系コマンドをターミナルで使用することは多いかなとは思いますが...シェルスクリプトも活用されていますか？
シェルスクリプトは、単なる「コマンドの羅列」ではなく、強力なプログラミング言語です。Linuxの世界では、複雑な処理を自動化し、システム管理者の日常業務を劇的に効率化するツールとも言えるでしょう。

このチートシートに記載されているコマンドやテクニックの多くは、シェルスクリプトの中だけでなく、UNIX/Linuxのコマンドラインでも直接使うことができます。例えばファイル操作コマンド（`find`, `grep`, `wc`など）、リダイレクト、パイプを使ったコマンドの連結、文字列操作、ジョブ制御などは日常的なシステム管理作業でも頻繁に活用されます。特に「便利なワンライナー集」セクションのコマンドは、スクリプトを書かなくても直接ターミナルで実行可能です。

この記事では、初めてShellScriptに触れる方から、より高度な技術を求めるベテランまで、全てのレベルの方々に役立つ情報をギュッと詰め込みました。あれ？これどうやって書くんだっけ？と思った際にいつでも戻ってきてください！

## 目次
- [他のチートシート](#他のチートシート)
- [他のシリーズ記事](#他のシリーズ記事)
- [基本的な構文](#基本的な構文)
- [変数](#変数)
- [条件分岐](#条件分岐)
- [ループ](#ループ)
- [関数](#関数)
- [コマンドライン引数](#コマンドライン引数)
- [ファイル操作](#ファイル操作)
- [文字列操作](#文字列操作)
- [配列](#配列)
- [数値計算](#数値計算)
- [プロセス制御](#プロセス制御)
- [エラーハンドリング](#エラーハンドリング)
- [デバッグテクニック](#デバッグテクニック)
- [シェルスクリプトのベストプラクティス](#シェルスクリプトのベストプラクティス)
- [便利なワンライナー集](#便利なワンライナー集)
- [上級テクニック](#上級テクニック)

## 他のチートシート
git/gh コマンド(gitコマンド以外にもgitの概念も書いてあります)

https://qiita.com/JavaLangRuntimeException/items/6b46551f56e0def76eba

lazygit

https://qiita.com/JavaLangRuntimeException/items/42087d09728d5739d73d

Docker コマンド(dockerコマンド以外にもdockerの概念の記事へのリンクもあります)

https://qiita.com/JavaLangRuntimeException/items/21f7c7bf3d143f821697

ステータスコード

https://qiita.com/JavaLangRuntimeException/items/ab1bc7b976ed2dfad91c

TypeScript

https://qiita.com/JavaLangRuntimeException/items/5894391c08e0d8e28389

Go/Gorm

https://qiita.com/JavaLangRuntimeException/items/d388717fc1436bc3ec9d

testing/gomock

https://qiita.com/JavaLangRuntimeException/items/bf521190f6f4d79e59fb


C#/.NET/Unity

https://qiita.com/JavaLangRuntimeException/items/7849b32bc223d4aa0247


Ruby・Ruby on Rails

https://qiita.com/JavaLangRuntimeException/items/42d935cf92c212f1c7ec

SQL

https://qiita.com/JavaLangRuntimeException/items/f038fbaccdd92fb0308a

Vim

https://qiita.com/JavaLangRuntimeException/items/0c68ab96ea198e0a7294

プルリクエスト・マークダウン記法チートシート

https://qiita.com/JavaLangRuntimeException/items/329eb92a47a07ff4dde8

ファイル操作コマンドチートシート

https://qiita.com/JavaLangRuntimeException/items/16f244606a73f7d106e4

VSCode Github Copilot拡張機能

https://qiita.com/JavaLangRuntimeException/items/be13dc3a346cf6e5ee44

OpenAI Assistants API

https://qiita.com/JavaLangRuntimeException/items/1a1abc01e8d7d05dce93

GitHub API

https://qiita.com/JavaLangRuntimeException/items/ab1bc7b976ed2dfad91c

変数・関数(メソッド)・クラス命名規則

https://qiita.com/JavaLangRuntimeException/items/b93865c448f69bcfca4a

## 他のシリーズ記事
**チートシート**
様々な言語，フレームワーク，ライブラリなど開発技術の使用方法，基本事項，応用事例を網羅し，手引書として記載したシリーズ

https://qiita.com/JavaLangRuntimeException/items/f038fbaccdd92fb0308a

> git/gh，lazygit，docker，vim，typescript，プルリクエスト/マークダウン，ステータスコード，ファイル操作，OpenAI AssistantsAPI，Ruby/Ruby on Rails のチートシートがあります．以下の記事に遷移した後，各種チートシートのリンクがあります.

**TypeScriptで学ぶプログラミングの世界**
プログラミング言語を根本的に理解するシリーズ

https://qiita.com/JavaLangRuntimeException/items/cadf49bb419076819963

**情報処理技術者試験合格への道 [IP・SG・FE・AP]**
情報処理技術者試験に出題されるコンピュータサイエンス用語の紹介や単語集

https://qiita.com/JavaLangRuntimeException/items/991be402099542ccb936

**IAM AWS User クラウドサービスをフル活用しよう！**
AWSのサービスを例にしてバックエンドとインフラ開発の手法を説明するシリーズです．

https://qiita.com/JavaLangRuntimeException/items/371a334f5a6e07035db5

**AWS UserのGCP浮気日記**
GCPの様子をAWSと比較して考えてみるシリーズ

https://qiita.com/JavaLangRuntimeException/items/527d99e774165a763180

**Project Gopher: Unlocking Go's Secrets**
Go言語や標準ライブラリの深掘り調査レポート

https://qiita.com/JavaLangRuntimeException/items/dc45b412d3fbd2ccb9e8

　

## 基本的な構文

### シェルスクリプトの基本構造

シェルスクリプトの基本的な構造です。最初の行には実行するシェルを指定し、コメントを書いてから実際のコマンドを記述します。

最初の行（シバン）でインタプリタを指定します：
```bash
#!/bin/bash
```

コメントを記述する場合：
```bash
# これはコメントです
```

基本的なメッセージ出力：
```bash
echo "Hello, World!"
```

最初の行 `#!/bin/bash` は「シバン（shebang）」と呼ばれ、このスクリプトを実行するインタプリタを指定します。Bashシェルを使用する場合は `/bin/bash` を指定します。

### 実行権限の設定

シェルスクリプトを実行するには、まずファイルに実行権限を付与する必要があります。実行権限を付与した後は、パスを指定して実行できます。

実行権限を付与：
```bash
chmod +x script.sh
```

スクリプトを実行：
```bash
./script.sh
```

### 基本的なコマンド出力

テキストを画面に出力するための基本コマンドです。echoは単純な出力に、printfはより複雑なフォーマットが必要な場合に使用します。

シンプルなテキスト出力：
```bash
echo "テキストを表示"
```

C言語風のフォーマット出力：
```bash
printf "フォーマット %s %d\n" "文字列" 42
```

### コメント

スクリプト内にメモや説明を残すためのコメント記法です。1行コメントと複数行コメントの2種類があります。

一行コメント：
```bash
# 一行コメント
```

複数行コメント：
```bash
: '
これは
複数行の
コメントです
'
```

## 変数

### 変数の定義と使用

シェルスクリプトで変数を定義し使用する基本的な方法です。変数名と値の間のイコール記号にはスペースを入れないように注意しましょう。

変数の定義（= の前後にスペースを入れないこと）：
```bash
NAME="Bash"
```

基本的な変数の使用：
```bash
echo "Hello, $NAME!"
```

波括弧を使った変数の使用（推奨）：
```bash
echo "Hello, ${NAME}!"
```

### 変数の型

Bashでは変数に明示的な型はありませんが、文脈によって異なる扱いになります。数値演算を行う場合は二重括弧を使った算術展開が必要です。

数値も文字列として扱われる：
```bash
NUMBER=42
```

文字列として出力される：
```bash
echo "$NUMBER + 10"
```

算術展開で数値として計算される：
```bash
echo $((NUMBER + 10))
```

### 特殊変数

シェルスクリプトには組み込みの特殊変数があり、スクリプト名や引数、プロセス情報などに簡単にアクセスできます。これらの変数は頻繁に使用されます。

スクリプト名を取得：
```bash
echo $0
```

最初の引数を取得：
```bash
echo $1
```

2番目の引数を取得：
```bash
echo $2
```

引数の数を取得：
```bash
echo $#
```

すべての引数を個別の文字列として取得：
```bash
echo $@
```

すべての引数を単一の文字列として取得：
```bash
echo $*
```

直前のコマンドの終了ステータスを取得：
```bash
echo $?
```

現在のシェルのプロセスIDを取得：
```bash
echo $$
```

最後にバックグラウンドで実行されたプロセスのIDを取得：
```bash
echo $!
```

### 環境変数

システム全体やユーザーセッション全体で利用できる変数です。システムの環境変数を参照したり、自分で環境変数を設定したりすることができます。

ホームディレクトリのパスを表示：
```bash
echo $HOME
```

実行ファイルの検索パスを表示：
```bash
echo $PATH
```

ユーザー名を表示：
```bash
echo $USER
```

環境変数として設定：
```bash
export MY_VAR="値"
```

### 読み取り専用変数

一度設定したら変更できない変数を作成します。定数として使用したい値に適しています。

読み取り専用変数を設定：
```bash
readonly CONSTANT="不変の値"
```

### コマンド置換

コマンドの実行結果を変数に格納したり、別のコマンドの一部として使用したりする方法です。現代的な構文と古い構文の両方があります。

コマンドの出力を変数に格納（現代的な構文）：
```bash
TODAY=$(date)
```

変数を使用して表示：
```bash
echo "今日は $TODAY です"
```

古い構文（互換性のために知っておくと良い）：
```bash
TODAY=`date`
```

## 条件分岐

### if 文

条件分岐の基本構造です。条件式が真の場合と偽の場合で異なる処理を実行できます。複数の条件を組み合わせることも可能です。

基本的なif文の構造：
```bash
if [ "$a" -eq "$b" ]; then
    echo "a と b は等しい"
elif [ "$a" -gt "$b" ]; then
    echo "a は b より大きい"
else
    echo "a は b より小さい"
fi
```

### 条件テスト

#### 数値比較

数値を比較するための演算子です。等値、大小関係などを調べることができます。文字列の比較とは異なる演算子を使用することに注意してください。

等しいかどうかをチェック：
```bash
[ "$a" -eq "$b" ]
```

等しくないかどうかをチェック：
```bash
[ "$a" -ne "$b" ]
```

大きいかどうかをチェック：
```bash
[ "$a" -gt "$b" ]
```

以上かどうかをチェック：
```bash
[ "$a" -ge "$b" ]
```

小さいかどうかをチェック：
```bash
[ "$a" -lt "$b" ]
```

以下かどうかをチェック：
```bash
[ "$a" -le "$b" ]
```

#### 文字列比較

文字列を比較するための演算子です。等値チェックだけでなく、空文字列のチェックやパターンマッチングもできます。二重角括弧構文ではより高度なマッチングが可能です。

文字列が等しいかどうかをチェック：
```bash
[ "$a" = "$b" ]
```

文字列が異なるかどうかをチェック：
```bash
[ "$a" != "$b" ]
```

文字列が空かどうかをチェック：
```bash
[ -z "$a" ]
```

文字列が非空かどうかをチェック：
```bash
[ -n "$a" ]
```

ワイルドカードマッチング（[[ ]] 構文のみ）：
```bash
[[ "$a" == *wild* ]]
```

正規表現マッチング（[[ ]] 構文のみ）：
```bash
[[ "$a" =~ regex ]]
```

#### ファイルテスト

ファイルやディレクトリの属性を検査するための演算子です。存在確認、種類確認、権限確認、ファイルの比較など、様々な用途に使用できます。

ファイルが存在するかどうかをチェック：
```bash
[ -e "$file" ]
```

通常のファイルであるかどうかをチェック：
```bash
[ -f "$file" ]
```

ディレクトリであるかどうかをチェック：
```bash
[ -d "$file" ]
```

読み取り可能であるかどうかをチェック：
```bash
[ -r "$file" ]
```

書き込み可能であるかどうかをチェック：
```bash
[ -w "$file" ]
```

実行可能であるかどうかをチェック：
```bash
[ -x "$file" ]
```

サイズが0より大きいかどうかをチェック：
```bash
[ -s "$file" ]
```

file1がfile2より新しいかどうかをチェック：
```bash
[ "$file1" -nt "$file2" ]
```

file1がfile2より古いかどうかをチェック：
```bash
[ "$file1" -ot "$file2" ]
```

### 基本的な算術演算

Bashでは整数の加減乗除やインクリメント・デクリメントなどの算術演算ができます。

変数の設定：
```bash
a=10
b=3
```

加算：
```bash
echo $((a + b))
```

減算：
```bash
echo $((a - b))
```

乗算：
```bash
echo $((a * b))
```

除算（整数）：
```bash
echo $((a / b))
```

剰余：
```bash
echo $((a % b))
```

べき乗：
```bash
echo $((a ** 2))
```

### case文

複数の値に対する条件分岐を効率的に記述できます。

```bash
case "$variable" in
    pattern1)
        echo "pattern1 に一致しました"
        ;;
    pattern2|pattern3)
        echo "pattern2 または pattern3 に一致しました"
        ;;
    *)
        echo "どのパターンにも一致しません"
        ;;
esac
```

## ループ

### for ループ

リストの要素を順番に処理したり、指定回数だけ処理を繰り返したりするための構文です。複数の書き方があり、目的に応じて使い分けます。

リストの各要素に対して処理：
```bash
for item in apple banana orange; do
    echo "Item: $item"
done
```

数値範囲の処理：
```bash
for i in {1..10}; do
    echo "$i"
done
```

増分を指定した範囲処理：
```bash
for i in {1..20..2}; do
    echo "$i"
done
```

C言語スタイルのforループ：
```bash
for ((i=0; i<10; i++)); do
    echo "$i"
done
```

### while ループ

条件が真である間、処理を繰り返し実行する構文です。カウンターを使った繰り返しや、特定の条件が満たされるまでの処理に使用します。無限ループを作ることも可能です。

条件付きの繰り返し処理：
```bash
count=0
while [ $count -lt 5 ]; do
    echo "Count: $count"
    ((count++))
done
```

無限ループ（Ctrl+Cで停止）：
```bash
while true; do
    echo "無限ループ"
    sleep 1
done
```

### until ループ

条件が真になるまで処理を繰り返す構文です。whileループの逆の動作をします。条件が偽の間は処理を繰り返し実行します。

条件が真になるまで処理を繰り返す：
```bash
count=5
until [ $count -eq 0 ]; do
    echo "Count down: $count"
    ((count--))
done
```

### ループ制御

ループの実行を制御するためのコマンドです。breakはループを途中で終了させ、continueは現在の繰り返しをスキップして次の繰り返しに進みます。

ループを抜ける例：
```bash
for i in {1..10}; do
    if [ $i -eq 5 ]; then
        break
    fi
    echo "$i"
done
```

次の繰り返しに進む例：
```bash
for i in {1..10}; do
    if [ $i -eq 5 ]; then
        continue
    fi
    echo "$i"
done
```

## 関数

### 関数の定義と呼び出し

スクリプト内で再利用可能なコードブロックを作成します。関数は定義してから呼び出して使用します。関数には引数を渡すことができ、特殊変数 $1, $2 などで参照できます。

関数の定義（functionキーワードを使用）：
```bash
function hello() {
    echo "Hello, $1!"
}
```

別の定義方法：
```bash
greet() {
    echo "Welcome, $1!"
}
```

関数の呼び出し：
```bash
hello "World"
greet "User"
```

### 戻り値

関数から値を返す方法です。Bashでは数値の終了ステータスを返す方法と、コマンド置換を使って文字列などのデータを返す方法があります。

戻り値（終了ステータス）を返す関数：
```bash
check_status() {
    if [ $1 -eq 0 ]; then
        return 0
    else
        return 1
    fi
}
```

関数の呼び出しと戻り値の確認：
```bash
check_status 0
if [ $? -eq 0 ]; then
    echo "成功しました"
fi
```

戻り値として文字列を返す関数：
```bash
get_date() {
    echo $(date +%Y-%m-%d)
}
```

関数の戻り値を変数に格納：
```bash
today=$(get_date)
echo "Today is $today"
```

### ローカル変数

関数内でのみ有効な変数を宣言する方法です。localキーワードを使うとその変数のスコープが関数内に限定され、グローバル変数との衝突を防ぎます。

ローカル変数とグローバル変数の使い分け：
```bash
function scope_test() {
    local local_var="ローカル変数"
    global_var="グローバル変数"
    echo "関数内: $local_var, $global_var"
}
```

関数実行：
```bash
scope_test
```

関数外での変数確認：
```bash
echo "関数外: $global_var"
echo "関数外: $local_var"
```

## コマンドライン引数

### 基本的な引数処理

シェルスクリプトにコマンドライン引数を渡し、それらを処理する基本的な方法です。特殊変数を使って引数の数や内容にアクセスできます。

スクリプト名の表示：
```bash
echo "スクリプト名: $0"
```

第1引数の表示：
```bash
echo "第1引数: $1"
```

第2引数の表示：
```bash
echo "第2引数: $2"
```

すべての引数の表示：
```bash
echo "すべての引数: $@"
```

引数の数の表示：
```bash
echo "引数の数: $#"
```

### getopts によるオプション処理

UNIXスタイルのコマンドラインオプション（-a、-bなど）を処理するための方法です。コロンはオプションに値が必要なことを示します。エラー処理も組み込まれています。

```bash
#!/bin/bash
while getopts ":a:b:c" opt; do
  case $opt in
    a)
      echo "オプション -a の値: $OPTARG"
      ;;
    b)
      echo "オプション -b の値: $OPTARG"
      ;;
    c)
      echo "オプション -c が指定されました"
      ;;
    \?)
      echo "不明なオプション: -$OPTARG" >&2
      exit 1
      ;;
    :)
      echo "オプション -$OPTARG には引数が必要です" >&2
      exit 1
      ;;
  esac
done
```

残りの引数を処理：
```bash
shift $((OPTIND-1))
echo "残りの引数: $@"
```

## ファイル操作

### ファイル読み込み

テキストファイルの内容を読み込み、処理する方法です。1行ずつ読み込んだり、フィールド区切り文字を指定してCSVなどの構造化データを処理したりできます。

1行ずつファイルを読み込む：
```bash
while read line; do
    echo "Line: $line"
done < file.txt
```

IFSを使ってCSVファイルを読み込む：
```bash
while IFS=, read -r col1 col2 col3; do
    echo "Column 1: $col1, Column 2: $col2, Column 3: $col3"
done < data.csv
```

### ファイル書き込み

ファイルにテキストデータを書き込むためのさまざまな方法です。上書きや追記、複数行のテキストブロック（ヒアドキュメント）などの書き込み方法があります。

ファイルに書き込み（上書き）：
```bash
echo "New content" > output.txt
```

ファイルに追記：
```bash
echo "Additional content" >> output.txt
```

ヒアドキュメントを使用した複数行の書き込み：
```bash
cat <<EOF > output.txt
これは複数行の
テキストを書き込む
方法です。
EOF
```

ヒアストリングを使用した書き込み：
```bash
cat <<< "単一文字列からの入力" > output.txt
```

### リダイレクト

コマンドの入出力をファイルにリダイレクトする方法です。標準出力、標準エラー出力、標準入力などを別のファイルや他のコマンドに接続できます。

標準出力をファイルにリダイレクト（上書き）：
```bash
command > output.txt
```

標準出力をファイルに追記：
```bash
command >> output.txt
```

標準エラー出力をファイルにリダイレクト：
```bash
command 2> error.txt
```

標準出力と標準エラー出力をファイルにリダイレクト：
```bash
command > output.txt 2>&1
```

標準出力と標準エラー出力をファイルにリダイレクト（短縮形）：
```bash
command &> output.txt
```

ファイルから標準入力を読み込む：
```bash
command < input.txt
```

## 文字列操作

### 文字列の長さ

文字列の文字数（長さ）を取得する方法です。変数名の前に#を付けることで簡単に文字数を取得できます。

文字列の長さを取得：
```bash
str="Hello, World!"
echo ${#str}
```

### 部分文字列の抽出

文字列の一部を取り出す方法です。開始位置と長さを指定したり、負のインデックスを使って末尾からの位置を指定したりできます。

インデックス7から末尾まで抽出：
```bash
str="Hello, World!"
echo ${str:7}
```

インデックス0から5文字抽出：
```bash
echo ${str:0:5}
```

末尾から6文字目から末尾まで抽出（スペース必須）：
```bash
echo ${str: -6}
```

末尾から6文字目から5文字抽出：
```bash
echo ${str: -6:5}
```

### 文字列置換

文字列置換は、文字列内の特定のパターンを検索し、新しいテキストに置き換える操作です。Bashでは `${変数名/パターン/置換テキスト}` の構文を使用して、様々な置換操作を行うことができます。

最初のマッチのみ置換：
```bash
str="Hello, World!"
echo ${str/World/Bash}
```

すべてのマッチを置換：
```bash
echo ${str//l/L}
```

先頭マッチを置換：
```bash
echo ${str/#Hello/Hi}
```

末尾マッチを置換：
```bash
echo ${str/%World!/People!}
```

### パターンマッチング

パターンマッチングは、ファイル名やパスなどの文字列から特定の部分を抽出したり削除したりするのに非常に便利です。

末尾の.txtを削除：
```bash
filename="document.txt"
echo ${filename%.txt}
```

末尾のドットと拡張子を削除：
```bash
echo ${filename%.*}
```

最初のドットより前を削除：
```bash
echo ${filename#*.}
```

パスからファイル名を抽出：
```bash
echo ${filename##*/}
```

パスの処理例：
```bash
path="/usr/local/bin/command"
echo ${path##*/}
```

ディレクトリパスを抽出：
```bash
echo ${path%/*}
```

### 大文字小文字変換

Bashでは文字列の大文字小文字を簡単に変換することができます。テキスト処理やユーザー入力の正規化に役立ちます。

すべて大文字に変換：
```bash
str="Hello, World!"
echo ${str^^}
```

すべて小文字に変換：
```bash
echo ${str,,}
```

先頭文字を大文字に変換：
```bash
echo ${str^}
```

先頭文字を小文字に変換：
```bash
echo ${str,}
```

## 配列

Bashでは、インデックス配列（数値インデックス）と連想配列（文字列キー）の2種類の配列を使用できます。配列は複数の関連するデータ要素を一つの変数に格納するのに役立ちます。

### 配列の宣言と操作

インデックス配列は0から始まるインデックスでアクセスする通常の配列で、連想配列はキーと値のペアを格納します。

配列の宣言：
```bash
fruits=("apple" "banana" "orange")
numbers=(1 2 3 4 5)
```

個別の要素を設定：
```bash
fruits[3]="grape"
```

配列要素の参照：
```bash
echo ${fruits[0]}
echo ${fruits[1]}
```

すべての要素を表示：
```bash
echo ${fruits[@]}
```

配列の長さ（要素数）を取得：
```bash
echo ${#fruits[@]}
```

配列のスライス：
```bash
echo ${fruits[@]:1:2}
```

配列の繰り返し処理：
```bash
for fruit in "${fruits[@]}"; do
    echo "I like $fruit"
done
```

配列への追加：
```bash
fruits+=("pear" "melon")
```

連想配列の宣言（Bash 4以降）：
```bash
declare -A user
```

連想配列の要素設定：
```bash
user[name]="John"
user[age]=30
```

連想配列の要素参照：
```bash
echo ${user[name]}
```

すべてのキーを表示：
```bash
echo ${!user[@]}
```

## 数値計算

### 基本的な算術演算

Bashでは整数の基本的な四則演算やべき乗などの算術演算を実行できます。算術展開を使用して計算を行います。

変数の設定：
```bash
a=10
b=3
```

加算：
```bash
echo $((a + b))
```

減算：
```bash
echo $((a - b))
```

乗算：
```bash
echo $((a * b))
```

除算（整数）：
```bash
echo $((a / b))
```

剰余：
```bash
echo $((a % b))
```

べき乗：
```bash
echo $((a ** 2))
```

### let コマンド

`let` コマンドは算術評価を行い、変数に値を代入するためのもう一つの方法です。

加算の結果を変数に代入：
```bash
let "c = a + b"
```

インクリメント：
```bash
let "a++"
```

複合代入演算子：
```bash
let "b+=5"
```

### bc コマンド（浮動小数点演算）

Bashは標準で整数演算しか対応していませんが、`bc`コマンドを使うと浮動小数点計算を行えます。`scale` パラメータで小数点以下の桁数を指定できます。

浮動小数点除算：
```bash
echo "scale=2; 5 / 2" | bc
```

平方根の計算：
```bash
result=$(echo "scale=4; sqrt(2)" | bc -l)
echo $result
```

## プロセス制御

Bashではプロセスの実行と管理を柔軟に制御できます。バックグラウンド実行、ジョブ制御、シグナル処理、そして複数のプロセスの同期など、様々なテクニックが用意されています。

### バックグラウンド実行

コマンドの末尾に `&` を付けることで、そのコマンドをバックグラウンドで実行できます。これにより、ターミナルを占有せずに長時間実行するタスクを処理できます。

コマンドをバックグラウンドで実行：
```bash
long_running_command &
```

最後に実行したバックグラウンドプロセスのPIDを取得：
```bash
echo $!
```

### ジョブ制御

シェルでは、複数のジョブを同時に管理することができます。ジョブはフォアグラウンドとバックグラウンドの間で移動させることが可能です。

現在のジョブリストを表示：
```bash
jobs
```

ジョブ1をフォアグラウンドに移動：
```bash
fg %1
```

ジョブ1をバックグラウンドで実行：
```bash
bg %1
```

ジョブ1を終了：
```bash
kill %1
```

### プロセスの終了シグナル

`kill` コマンドは、指定したプロセスにシグナルを送信してプロセスを終了させるために使用します。

プロセスを強制終了 (SIGKILL)：
```bash
kill -9 $PID
```

プロセスに正常終了を要求 (SIGTERM)：
```bash
kill -15 $PID
```

### トラップ

`trap` コマンドを使用すると、スクリプトが特定のシグナルを受信した際に実行するコードを定義できます。これはスクリプトの終了時にリソースをクリーンアップするのに特に便利です。

スクリプト終了時やCtrl+Cで中断された時の処理を定義：
```bash
trap "echo 'スクリプトが終了しました'; cleanup" EXIT
trap "echo '中断されました'; exit 1" INT TERM
```

クリーンアップ関数の定義：
```bash
cleanup() {
    echo "一時ファイルを削除しています..."
}
```

### 待機

`wait` コマンドは、バックグラウンドプロセスの完了を待機するために使用します。特定のプロセスや、すべてのバックグラウンドプロセスの完了を待つことができます。

複数のコマンドをバックグラウンドで実行：
```bash
command1 &
command2 &
```

すべてのバックグラウンドプロセスの完了を待つ：
```bash
wait
```

特定のプロセスの完了を待つ：
```bash
wait $PID
```

## エラーハンドリング

堅牢なシェルスクリプトを作成するには、エラーを適切に処理する必要があります。Bashには様々なエラーハンドリング手法があり、これらを使ってスクリプトの信頼性を高めることができます。

### エラーチェック

コマンド実行後の終了ステータス（`$?`）を確認することで、コマンドが成功したか失敗したかを判断できます。0は成功、1～255は何らかのエラーを示します。

コマンドの実行：
```bash
command
```

終了ステータスをチェック：
```bash
if [ $? -ne 0 ]; then
    echo "エラーが発生しました"
    exit 1
fi
```

### エラーが発生したら終了

`set -e` オプションを設定すると、コマンドがゼロ以外の終了ステータスを返した時点でスクリプトの実行を終了します。

エラーが発生したらスクリプトを終了：
```bash
set -e
```

### エラー出力

エラーメッセージは標準エラー出力（stderr）に書き込むのが良い習慣です。これにより、通常の出力とエラーメッセージを区別することができます。

標準エラー出力に書き込む：
```bash
echo "エラーメッセージ" >&2
```

### コマンドの失敗をチェック

論理演算子 `&&` と `||` を使用して、コマンドの成功または失敗に基づいて次のコマンドを実行するかどうかを制御できます。

コマンドが成功した場合だけ次のコマンドを実行：
```bash
command1 && command2
```

コマンドが失敗した場合だけ次のコマンドを実行：
```bash
command1 || command2
```

## デバッグテクニック

### デバッグモード

すべてのコマンドを実行前に表示：
```bash
#!/bin/bash -x
```

スクリプト内の特定の部分だけデバッグモード開始：
```bash
set -x
```

デバッグモード終了：
```bash
set +x
```

### シェルオプション

エラーが発生したら終了：
```bash
set -e
```

未定義の変数を使用したらエラー：
```bash
set -u
```

パイプの一部が失敗したら全体を失敗とする：
```bash
set -o pipefail
```

入力行を表示：
```bash
set -v
```

### トレース機能

デバッグ情報のフォーマットを設定：
```bash
PS4='+ ${BASH_SOURCE}:${LINENO}:${FUNCNAME[0]}: '
```

デバッグモードを有効化：
```bash
set -x
```

### デバッグ変数の表示

デバッグ用の変数表示関数：
```bash
dump_vars() {
    echo "DEBUG: var1=$var1, var2=$var2"
}
```

## シェルスクリプトのベストプラクティス

### 変数の引用符

変数を適切に引用：
```bash
name="John Doe"
echo "$name"
```

引用符なしは危険：
```bash
echo $name
```

### コマンド存在チェック

コマンドの存在確認：
```bash
if ! command -v git &> /dev/null; then
    echo "gitコマンドが見つかりません"
    exit 1
fi
```

### デフォルト値の設定

第1引数がなければデフォルト値を使用：
```bash
name=${1:-"Guest"}
```

### 安全な一時ファイル

一時ファイルの作成：
```bash
temp_file=$(mktemp /tmp/script.XXXXXX)
```

終了時のクリーンアップ設定：
```bash
trap "rm -f $temp_file" EXIT
```

### スクリプトのパス取得

スクリプトのディレクトリパスを取得：
```bash
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
```

### 関数の戻り値チェック

関数の実行：
```bash
function_name
```

戻り値をチェック：
```bash
if [ $? -eq 0 ]; then
    echo "関数は成功しました"
else
    echo "関数は失敗しました"
fi
```

## 便利なワンライナー集

### ファイル操作

特定の拡張子のファイルを検索：
```bash
find . -name "*.txt" -type f
```

ファイル内の特定のテキストを検索：
```bash
grep -r "検索文字列" .
```

直近で変更されたファイルを表示：
```bash
find . -type f -mtime -7 | sort -r
```

大きなファイルを検索：
```bash
find . -type f -size +100M
```

重複ファイルを検索：
```bash
find . -type f -exec md5sum {} \; | sort | uniq -d -w 32
```

### システム情報

ディスク使用量を表示：
```bash
df -h
```

ディレクトリサイズを表示：
```bash
du -sh /path/to/dir
```

メモリ使用量を表示：
```bash
free -m
```

システム負荷を表示：
```bash
uptime
```

### テキスト処理

ファイルの行数をカウント：
```bash
wc -l file.txt
```

特定の列を抽出（スペース区切り）：
```bash
awk '{print $1, $3}' file.txt
```

特定の列を抽出（カンマ区切り）：
```bash
cut -d, -f1,3 file.csv
```

テキストの置換：
```bash
sed 's/old/new/g' file.txt
```

重複行の削除：
```bash
sort file.txt | uniq
```

## 上級テクニック

### プロセス置換

二つのファイルの差分行を抽出：
```bash
grep -vf <(sort file1.txt) <(sort file2.txt)
```

複数のコマンド出力を一つのwhileループで処理：
```bash
while read line; do
    echo "Processing: $line"
done < <(command1; command2)
```

### 名前付きパイプ

名前付きパイプの作成：
```bash
mkfifo mypipe
```

コマンド1の出力をパイプに送信：
```bash
command1 > mypipe &
```

パイプからコマンド2に入力：
```bash
command2 < mypipe
```

パイプの削除：
```bash
rm mypipe
```

### コプロセス

コプロセスの作成：
```bash
coproc mycoproc { grep "pattern" -f -; }
```

コプロセスに入力送信：
```bash
echo "text with pattern" >&"${mycoproc[1]}"
```

コプロセスから出力読み取り：
```bash
read line <&"${mycoproc[0]}"
echo "$line"
```

### シェルスクリプトのプロファイリング

プロファイリング情報付きでスクリプト実行：
```bash
PS4='+ $(date "+%s.%N") $(basename ${BASH_SOURCE}):${LINENO}: ' bash -x ./script.sh
```

### シグナルハンドリングの高度なテクニック

クリーンアップ関数の定義：
```bash
cleanup() {
    rm -f "$tempfile"
    echo "Cleanup complete"
}
```

複数のシグナルをキャプチャ：
```bash
trap cleanup EXIT INT TERM HUP
```

---

シェルスクリプトは単なるコマンドの連続ではなく、豊かな表現力を持ったプログラミング言語です。このチートシートがあなたのシェルスクリプトの冒険において、頼もしい地図となることを願っています！

何か新しいテクニックを学んだり、スクリプトを効率化したりするたびに、このチートシートに戻ってきてください。常に学び続け、シェルスクリプトの魔法を使いこなしましょう！

## 参考文献
- [Bash 公式マニュアル](https://www.gnu.org/software/bash/manual/)
- [The Linux Documentation Project - Advanced Bash-Scripting Guide](https://tldp.org/LDP/abs/html/)
- [UNIX/Linux コマンド - シェルスクリプト入門 (Japanese)](https://shellscript.sunone.me/)
- [Bash スクリプティング入門 - 日本語解説](https://qiita.com/lrf141/items/4432b13a24d528a5648a)
- [Ryuichirou Suzuki - シェルスクリプトの基本](https://zenn.dev/ryo_suzuki/articles/6e63bf0c7607b4)
