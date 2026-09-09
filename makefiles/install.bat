ren ..\..\..\..\target\ev3_gcc\dmloader\app\Makefile.lum Makefile.lum.sav
ren ..\..\..\common\Makefile.workspace Makefile.workspace.sav

copy /y Makefile.lum ..\..\..\..\target\ev3_gcc\dmloader\app
copy /y Makefile.workspace ..\..\..\common

dir ..\..\..\..\target\ev3_gcc\dmloader\app
dir ..\..\..\common

echo install finished.
pause