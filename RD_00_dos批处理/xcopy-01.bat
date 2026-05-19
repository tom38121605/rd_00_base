echo off 
echo start copy .... %time% 

set "src=C:\workspace\ommo_eval_sdk_v0.21.1-channel_hopping\ommo_service"
set "dst=F:\logs"

xcopy    "%src%\hardware_logs_*\*"             "%dst%\"        /E /H /Y  /C /I
copy     "%src%\data\*.txt"    "%dst%\"        /y


echo xcopy backup success! ...%time% 
pause 



rem /E 复制所有子目录
rem /H 复制隐藏文件
rem /Y 覆盖不提示
rem /C 出错继续
rem /D 只复制新文件（增量备份）
rem /I：自动识别为文件夹