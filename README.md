# ev3-template

これは、[Toppers/EV3RTプロジェクト](https://toppers.jp/trac_user/ev3pf/wiki/WhatsEV3RT)の成果を使い、C言語でLEGO Mindstorms EV3を動作させるときに便利なユーティリティの一群です。

- デバッグに使うLCDモニター用のprintf
- 一般的なモーターペアを使ったステアリング関数
- タンク関数
- ライントレースのサンプル

などがあります。

## 動作環境

WSLのUbuntu 22と26.04-LTSくらいで動作確認

## EV3RTのインストール

EV3-RTのプロジェクトをインストール、hrp3のtarファイルを展開した後、`hrp3/sdk/workspace` 以下に本プロジェクトを展開してください。

## あなたのプロジェクトを作るには

`app.c` の `main_task` を編集します。サブルーチンのファイルを分けたいときは、`Makefile.inc` に追記します。
タスクやサイクルハンドラ等を増やす方法は、EV3-RTやSpike-RTの関連ドキュメントを探してください。

## あなたのプロジェクトをビルドするには

```bash
cd hrp3/sdk/workspace
make app=ev3-template   # 本プロジェクトのディレクトリ名
```

## EV3にダウンロード

Windows側で `download.bat` を実行します。
EV3をUSBドライブとしてマウントしているときにコピーしているだけなので、変数 `drive` を適宜変えてください（初期値は `D:`）。

## おまけ：makeの高速化

EV3-RTプロジェクトのMakefileは、appをビルドするときに変更のあるファイルだけをビルドするようになっていません。全コンパイルし直しです。
コンパイルの時間を気にする人は、`makefiles` 以下にあるバッチファイルを実行して、EV3-RTプロジェクトのMakefileを入れ替えてみるとよいことがあるかもしれません。
