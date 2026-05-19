echo off 
echo start copy .... %time% 

set "src=C:\workspace\ommo_eval_sdk_v0.21.1-channel_hopping\ommo_service"
set "dst=F:\logs\"

for /d %%a in ("%src%\hardware_logs*") do (
    echo 正在复制：%%~nxa
    xcopy "%%a\*" "%dst%\%%~nxa\" /E /H /Y /I /D
)

copy     "%src%\data\*.txt"    "%dst%\"        /y

echo xcopy backup success! ...%time% 
pause 



rem /E 复制所有子目录
rem /H 复制隐藏文件
rem /Y 覆盖不提示
rem /C 出错继续
rem /D 只复制新文件（增量备份）
rem /I：自动识别为文件夹
rem %%a = 完整路径，例如 D:\test\hardware_logs_202501
rem %%~nxa = 只拿最后的文件夹名 → hardware_logs_202501