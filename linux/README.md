# Linux (Ubuntu 22.04) 課題

## 目標
- 「Docker」「ROS 2」課題などで利用するLinux (Ubuntu 22.04) の基礎知識習得
- Linux (Ubuntu 22.04) 環境のセットアップ

## 課題内容

### 1. WSL (Windows Subsystem for Linux 2) で Ubuntu 22.04 を実行する

研究室で利用されているLinux (Ubuntu 22.04) をWSL2 (Windows Subsystem for Linux 2) 上で実行します。今後使用するツール等との互換性を考慮し、最新バージョン(24.04) ではなく、22.04を推奨します。なお、有線ネットワークでのインストールを推奨します。

#### インストール方法
以下の記事を参考にしてください：
- [Microsoft公式ドキュメント](https://learn.microsoft.com/ja-jp/windows/wsl/install)
- [Qiita記事](https://qiita.com/zakoken/items/61141df6aeae9e3f8e36)

#### 理解すべき内容
- **Linux とは何か？Ubuntu とは何か？それぞれの違いは？**
  - Linux：オープンソースのオペレーティングシステムカーネル
  - Ubuntu：Linuxカーネルを基にしたディストリビューション（配布版）
  - LinuxはOSの中核部分、UbuntuはLinuxを使いやすくパッケージ化したもの

- **仮想マシンとは何か？WSL とは何か？**
  - 仮想マシン：物理マシン上で別のOSを動作させる技術
  - WSL：Windows上でLinux環境を軽量に実行できるマイクロソフトの技術

- **実PCにUbuntuをインストールするにはどうすればよいか？**
  - ISOファイルをダウンロードしてUSBブートディスクを作成
  - BIOSでブート順序を変更してUSBから起動
  - インストーラーの指示に従ってパーティション設定・インストール実行

### 2. Linux コマンドを学ぶ

Linux では、GUI（グラフィカルなインタフェース）ではなくCUI/CLI（文字によるインタフェース）で操作を行うことが多いです。この課題では、CUI操作によりLinuxコマンドを実行し、ディレクトリ・ファイル操作とソフトウェアのインストール方法を学びます。

#### a. Linux コマンドを使ってディレクトリ・テキストファイルを作成する

Linuxコマンドを用いてホームディレクトリに以下のディレクトリ・テキストファイルを作成します。

**作成するテキストファイル (hello.txt)**
- 場所: `~/task2-a/hello.txt`
- 内容: `Hello, Linux!`

##### 実行手順

1. ホームディレクトリに移動
```bash
cd ~
```

2. task2-a ディレクトリを作成
```bash
mkdir task2-a
```

3. ディレクトリに移動
```bash
cd task2-a
```

4. hello.txt ファイルを作成し、内容を書き込み
```bash
echo "Hello, Linux!" > hello.txt
```

または nanoエディタを使用：
```bash
nano hello.txt
```

5. ファイルの確認
```bash
cat hello.txt
```

##### 理解すべき内容

- **ファイルの一覧を表示するにはどうすればよいか？**
```bash
ls        # 基本的なファイル一覧
ls -l     # 詳細情報付きファイル一覧
ls -la    # 隠しファイルも含む詳細ファイル一覧
```

- **他の場所にファイルをコピー・移動したい場合はどうすればよいか？**
```bash
cp hello.txt ~/backup/hello.txt    # ファイルコピー
mv hello.txt ~/backup/hello.txt    # ファイル移動
```

- **ファイル名を変更したい場合にはどうすればよいか？**
```bash
mv hello.txt greeting.txt    # ファイル名変更
```

- **誤って作成したファイルを削除するにはどうすればよいか？**
```bash
rm hello.txt        # ファイル削除
rm -rf directory/   # ディレクトリを中身ごと削除（注意！）
```

#### b. aptコマンドを使ってリポジトリからアプリケーションをインストールする

Linuxではリポジトリと呼ばれるソフトウェアサーバからアプリケーションをインストールして使用することが一般的です。sudoコマンドとaptコマンドを使って"sl"というアプリケーションをインストールし、実行します。

##### 実行手順

1. パッケージリストを更新
```bash
sudo apt update
```

2. sl アプリケーションをインストール
```bash
sudo apt install sl
```

3. sl を実行
```bash
sl
```

##### 理解すべき内容

- **"sudo" とは何か？ "root" とは何か？**
  - sudo：一時的に管理者権限でコマンドを実行するためのコマンド
  - root：Linuxシステムの最高権限ユーザー（管理者）

- **他のアプリケーションをインストールするにはどうすればよいか？**
```bash
sudo apt install <パッケージ名>
```

- **インストールしたアプリケーションをアップデートするにはどうすればよいか？**
```bash
sudo apt update     # パッケージリスト更新
sudo apt upgrade    # 全パッケージアップデート
sudo apt upgrade <パッケージ名>  # 特定パッケージアップデート
```

- **インストールしたアプリケーションをアンインストールするにはどうすればよいか？**
```bash
sudo apt remove <パッケージ名>      # アプリケーションのみ削除
sudo apt purge <パッケージ名>       # 設定ファイルも含めて削除
sudo apt autoremove               # 不要な依存関係を削除
```

### 3. シェルスクリプトを作成する

これまでに学んだ知識を活かし、シェルスクリプトを作成します。シェルスクリプトとは、複数のLinuxコマンドを記述したスクリプトであり、これまでに学んだLinuxコマンドを動的に実行したり、繰り返したりすることができます。

#### シェルスクリプトで実行する内容

1. "----start----" とターミナルに表示
2. ホームディレクトリ直下に"task3"ディレクトリを作成
3. 課題2-aで作成したテキストファイルを"task3"にコピー
4. コピーしたテキストファイルの名前をシェルスクリプトに引数で与えた名前に変更
5. コピーしたテキストファイルの内容をターミナルに表示（catコマンドを使う）
6. "----end----" とターミナルに表示

#### 実行手順

1. シェルスクリプトファイルを実行可能にする
```bash
chmod +x task3.sh
```

2. シェルスクリプトを実行（引数として新しいファイル名を指定）
```bash
./task3.sh newfile.txt
```

#### 理解すべき内容

- **シェルスクリプトで処理を繰り返すにはどうすればよいか？**
```bash
# for文
for i in {1..5}; do
    echo "Count: $i"
done

# while文
counter=1
while [ $counter -le 5 ]; do
    echo "Count: $counter"
    counter=$((counter + 1))
done
```

- **シェルスクリプトで処理を条件分岐するにはどうすればよいか？**
```bash
# if文
if [ $# -eq 0 ]; then
    echo "引数が指定されていません"
else
    echo "引数: $1"
fi

# case文
case $1 in
    "start")
        echo "開始します"
        ;;
    "stop")
        echo "停止します"
        ;;
    *)
        echo "不明なコマンド"
        ;;
esac
```

- **Linuxにおける「実行権限」とは何か？**
  - ファイルを実行できる権限
  - `chmod +x filename` で実行権限を付与

- **Linuxにおける「ファイルの権限」とは何か？**
  - 読み取り権限（r）：ファイルの内容を読める
  - 書き込み権限（w）：ファイルの内容を変更できる
  - 実行権限（x）：ファイルを実行できる
  - 所有者、グループ、その他のユーザーごとに権限を設定可能

```bash
# 権限の確認
ls -l filename

# 権限の変更例
chmod 755 filename  # 所有者：読み書き実行、グループ・その他：読み実行
chmod u+x filename  # 所有者に実行権限を追加
chmod g-w filename  # グループの書き込み権限を削除
```

## 注意事項

⚠️ **重要な注意点**
WSL2ではWindows環境とLinux環境は完全には分離されていません。例えば、Windowsのファイルシステムには"/mnt/c"などを介してアクセスすることができます。

破壊的なコマンドを実行した場合、Windows環境にも影響が及ぶ可能性があるため、コマンドの意味をよく理解してから実行する必要があります。

## 参考資料

- [Linux command入門](https://jellyware.jp/kurage/movidius/c04command_nano.html)
- [Microsoft WSL公式ドキュメント](https://learn.microsoft.com/ja-jp/windows/wsl/install)
