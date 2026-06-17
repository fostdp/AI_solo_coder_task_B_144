@echo off
chcp 65001 >nul
echo ========================================
echo 古代双辕车转向机构仿真分析系统
echo ========================================
echo.
echo 请选择要启动的服务：
echo [1] 启动后端服务 (FastAPI)
echo [2] 启动传感器模拟器
echo [3] 初始化 InfluxDB
echo [4] 启动全部服务
echo [0] 退出
echo.
set /p choice=请输入选项：

if "%choice%"=="1" goto backend
if "%choice%"=="2" goto simulator
if "%choice%"=="3" goto init_influx
if "%choice%"=="4" goto all
if "%choice%"=="0" goto end

:backend
echo.
echo 启动后端服务...
cd /d "%~dp0backend"
python main.py
goto end

:simulator
echo.
echo 启动传感器模拟器...
cd /d "%~dp0scripts"
python sensor_simulator.py
goto end

:init_influx
echo.
echo 初始化 InfluxDB...
cd /d "%~dp0scripts"
python init_influxdb.py
goto end

:all
echo.
echo 启动全部服务...
cd /d "%~dp0backend"
start "后端服务" python main.py
timeout /t 3 /nobreak >nul
cd /d "%~dp0scripts"
start "传感器模拟器" python sensor_simulator.py
echo.
echo 服务已启动！
echo 访问地址: http://localhost:8000/frontend/index.html
echo.
pause
goto end

:end
echo.
echo 退出完成。
pause
