@echo off
setlocal

set drive=d:

if not exist %drive%\uImage (
    echo �h���C�u%drive%������܂���B�ϐ�drive��ύX���邩�AEV3-RT��USB�ڑ����Ă�������
    goto :end
)

if not exist ..\app (
    echo app���R���p�C������Ă��܂���B
    goto :end
)

echo app���R�s�[���܂�
copy ..\app %drive%\ev3rt\apps
echo app���R�s�[���܂����BEV3��USB�ڑ����O���Ă��������B

:end
pause
endlocal