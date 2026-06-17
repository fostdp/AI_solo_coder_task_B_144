@echo off
chcp 65001 >nul
echo ========================================================
echo  古代双辕车仿真系统 - 微服务启动脚本 v2.0
echo ========================================================
echo.

echo [1/6] 检查Python依赖...
python -c "import redis" 2>nul || (echo 正在安装 redis... && pip install redis)
python -c "import fastapi" 2>nul || (echo 正在安装 fastapi... && pip install fastapi uvicorn)
python -c "import influxdb_client" 2>nul || (echo 正在安装 influxdb_client... && pip install influxdb-client)
python -c "import paho.mqtt" 2>nul || (echo 正在安装 paho-mqtt... && pip install paho-mqtt)
echo.

echo [2/6] 启动 API 网关 (端口 8000)...
start "API_Gateway" cmd /k "cd /d %~dp0 && set PYTHONPATH=services && python -m uvicorn api_gateway.main:app --host 0.0.0.0 --port 8000 --reload"
timeout /t 2 /nobreak >nul

echo [3/6] 启动 DTU 数据接收器 (端口 8001)...
start "DTU_Receiver" cmd /k "cd /d %~dp0 && set PYTHONPATH=services && python -m uvicorn dtu_receiver.main:app --host 0.0.0.0 --port 8001 --reload"
timeout /t 2 /nobreak >nul

echo [4/6] 启动 转向仿真服务 (端口 8002)...
start "Steering_Simulator" cmd /k "cd /d %~dp0 && set PYTHONPATH=services && python -m uvicorn steering_simulator.main:app --host 0.0.0.0 --port 8002 --reload"
timeout /t 2 /nobreak >nul

echo [5/6] 启动 稳定性分析服务 (端口 8003)...
start "Stability_Analyzer" cmd /k "cd /d %~dp0 && set PYTHONPATH=services && python -m uvicorn stability_analyzer.main:app --host 0.0.0.0 --port 8003 --reload"
timeout /t 2 /nobreak >nul

echo [6/6] 启动 告警MQTT服务 (端口 8004)...
start "Alarm_MQTT" cmd /k "cd /d %~dp0 && set PYTHONPATH=services && python -m uvicorn alarm_mqtt.main:app --host 0.0.0.0 --port 8004 --reload"
timeout /t 2 /nobreak >nul

echo.
echo ========================================================
echo  所有服务启动完成！
echo ========================================================
echo.
echo  服务列表:
echo    API 网关     : http://localhost:8000  (对外入口)
echo    DTU 接收器   : http://localhost:8001
echo    转向仿真     : http://localhost:8002
echo    稳定性分析   : http://localhost:8003
echo    告警MQTT     : http://localhost:8004
echo.
echo  前端页面      : http://localhost:8000/frontend/index.html
echo  健康检查      : http://localhost:8000/health
echo.
echo  通信中间件:
echo    Redis 通道   : chariot:*
echo    MQTT 主题    : chariot/alerts, chariot/data
echo.
echo  按任意键启动传感器模拟器...
pause >nul

echo.
echo 启动传感器模拟器...
start "Sensor_Simulator" cmd /k "cd /d %~dp0 && python scripts/sensor_simulator.py"

echo.
echo 系统全部启动完毕！
pause
