@echo off

rem Tcl script created by Tang Dynasty 6.2.1 168116
rem Copyright (c) 2012-2025 Anlogic Inc.
rem
rem 由 TD 生成后做过一处手工调整：安装路径改读环境变量 TD_HOME，
rem 未设置时回退到本机默认安装位置，避免把某台机器的绝对路径写进仓库。

set HD_SDIR=%~dp0
cd /d "%HD_SDIR%"

if "%TD_HOME%"=="" set TD_HOME=C:\Anlogic\TD_6.2.1_Engineer_6.2.168.116

if not exist "%TD_HOME%\bin\td_commands_prompt.exe" (
    echo [ERROR] 未找到 TD 命令行：请设置环境变量 TD_HOME 指向 TD 安装目录，
    echo         例如 set TD_HOME=C:\Anlogic\TD_6.2.1_Engineer_6.2.168.116
    pause
    exit /b 1
)

"%TD_HOME%\bin\td_commands_prompt.exe" "%TD_HOME%\doc\scripts\DefaultFlow.tcl"
pause
