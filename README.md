# ev3-template

これは、Toppers/EV3RTプロジェクト
https://toppers.jp/trac_user/ev3pf/wiki/WhatsEV3RT
の成果を使い、C言語でLEGO Mindstorms EV3を動作させるときに便利なユーティリティの一群です。

デバッグに使うprintf
一般的なモーターペアを使ったステアリング関数
などがあります。

【動作環境】
WSLのUbuntu たしか22くらいで動作確認

【EV3RTのインストール】
sudo ./ev3rt_install.sh

【あなたのプロジェクトを作るには】
app.cのmain_taskに書いてください。

【あなたのプロジェクトをビルドするには】
sh makeapp.sh

【EV3にダウンロード】
Windows側でdownload.batを実行
