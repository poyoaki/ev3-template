@echo off
setlocal

set drive=d:

if not exist %drive%\uImage (
    echo ドライブ%drive%がありません。変数driveを変更するか、EV3-RTをUSB接続してください
    goto :end
)

if not exist ..\app (
    echo appがコンパイルされていません。
    goto :end
)

echo appをコピーします
copy ..\app %drive%\ev3rt\apps
echo appをコピーしました。EV3のUSB接続を外してください。

:end
pause
endlocal