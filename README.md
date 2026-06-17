# 秦汉双辕车转向机构仿真与操控稳定性分析系统 v2.0

## 项目简介

本系统是为交通史研究团队设计的秦汉双辕车复原研究平台，基于微服务架构，实现古代双辕车的转向机构仿真、操控稳定性分析和实时告警推送。

---

## 系统架构

### 架构图

```
┌─────────────────────────────────────────────────────────────────────────────────────┐
│                                      客户端层                                         │
│  ┌─────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌──────────────────┐   │
│  │  Web浏览器   │  │   MQTT客户端    │  │  InfluxDB UI    │  │   移动端APP      │   │
│  │  (Nginx)    │  │  (告警订阅)     │  │  (数据查询)     │  │  (数据展示)      │   │
│  └──────┬──────┘  └────────┬────────┘  └────────┬────────┘  └────────┬─────────┘   │
│         │ :80               │                   │                   │               │
└─────────┼───────────────────┼───────────────────┼───────────────────┼───────────────┘
          │                   │                   │                   │
┌─────────┴───────────────────┼───────────────────┼───────────────────┼───────────────┐
│  API Gateway (8000)         │                   │                   │               │
│  REST + WebSocket           │                   │                   │               │
└──────┬─────────┬────────────┘                   │                   │               │
       │         │                                 │                   │               │
       │         └─────────────────────────────────┼───────────────────┘               │
       │                                           │                                   │
┌──────┴──────────────┐  Pub/Sub  ┌──────────────────┐  Pub/Sub  ┌──────────────────┐  │
│  dtu_receiver       │──────────▶│ steering_simulator│──────────▶│ stability_analyzer│  │
│  (8001)             │  传感器   │  (8002)           │  转向结果  │  (8003)           │  │
│  数据采集校验        │  数据     │  四杆机构计算     │           │  变重心动力学     │  │
└─────────────────────┘           └─────────┬─────────┘           └─────────┬────────┘  │
                                            │                               │           │
                                            └──────────────┬────────────────┘           │
                                                           │                            │
                                                    ┌──────┴──────┐                     │
                                                    │ alarm_mqtt  │                     │
                                                    │  (8004)     │                     │
                                                    │  告警检测    │                     │
                                                    │  MQTT推送    │───┐                 │
                                                    │  InfluxDB存储│   │ MQTT          │
                                                    └──────────────┘   │ 1883           │
                                                                         ▼                 │
                                                       ┌──────────────────────────┐      │
                                                       │  Mosquitto MQTT Broker   │      │
                                                       │  (告警/数据推送)         │      │
                                                       └──────────────────────────┘      │
                                                                                           │
┌───────────────────────────────────────────────────────────────────────────────────────────┤
│  基础设施层                                                                              │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────┐  ┌──────────────────────┐      │
│  │   Redis     │  │   InfluxDB   │  │   Mosquitto     │  │      Nginx           │      │
│  │  (缓存)     │  │  (时序存储)  │  │  (MQTT Broker)  │  │  (前端+反向代理)      │      │
│  └─────────────┘  └──────────────┘  └─────────────────┘  └──────────────────────┘      │
└───────────────────────────────────────────────────────────────────────────────────────────┘
```

### 微服务清单

| 服务名称 | 端口 | 职责 |
|---------|------|------|
| `api_gateway` | 8000 | REST接口 + WebSocket实时推送 + 服务聚合 |
| `dtu_receiver` | 8001 | 传感器数据接收、校验、Redis发布 |
| `steering_simulator` | 8002 | 四杆机构运动学 + 阿克曼几何计算 |
| `stability_analyzer` | 8003 | 变重心动力学 + 侧翻/横摆稳定性评估 |
| `alarm_mqtt` | 8004 | 告警阈值检测 + MQTT推送 + InfluxDB存储 |

### 数据流

```
传感器POST /api/sensor/data → [dtu_receiver] 校验
  ↓ Redis chariot:validated:sensor
    [steering_simulator] 四杆机构计算 → SteeringResult
      ↓ Redis chariot:result:steering
        [stability_analyzer] 稳定性分析 → StabilityResult
          ↓ Redis chariot:result:stability
            [alarm_mqtt] 告警检测 → MQTT推送 + InfluxDB存储
              ↓ Redis chariot:alerts
                [api_gateway] WebSocket广播 → 前端实时展示
```

---

## 核心特性

### 1. 转向机构仿真
- **阿克曼转向几何**：`cot(δo) - cot(δi) = T/L`
- **四杆机构运动学**：Freudenstein方程求解、传动角约束、死点避让、连杆干涉检测
- **三级降级策略**：四杆求解 → 95%最大安全角 → 理想阿克曼公式

### 2. 操控稳定性分析
- **变重心动力学**：货物质量合成重心、平行轴定理计算转动惯量、弹簧-阻尼模型模拟货物动态位移
- **横摆角速度**：自行车模型 + 不足转向系数修正
- **侧翻风险评估**：SSF静态稳定系数、载荷转移率、内轮抬升检测

### 3. 告警系统
- 侧倾角 > 20° → critical
- 滑移率 > 0.8 或 < 0.05 → warning/info
- 路面摩擦系数 < 0.3 → warning
- 侧翻风险 > 70% → critical
- 稳定性指数 < 0.3 → critical
- MQTT主题推送 + 5分钟冷却抑制

### 4. 数据存储与降采样
| 数据级别 | 粒度 | 保留时间 | 存储桶 |
|---------|------|----------|--------|
| 原始数据 | 1分钟 | 7天 | `chariot_data` |
| 小时级降采样 | 1小时 | 7天 | `chariot_data_downsampled_1h` |
| 天级降采样 | 1天 | 365天 | `chariot_data_downsampled_1d` |

### 5. 前端特性
- Three.js WebGL 3D渲染 + GPU InstancedMesh实例化
- 转向机构连杆实时动画
- 车轮轨迹动态线条
- Canvas 2D连杆示意图
- WebSocket实时数据推送

---

## 快速开始

### 环境要求
- Docker >= 20.10
- Docker Compose >= 2.0
- 至少 4GB 内存
- 至少 10GB 磁盘空间

### 一键部署

```bash
# 1. 克隆项目
git clone <repository-url>
cd chariot-simulation

# 2. 配置环境变量（可选，默认即可）
cp .env.example .env
vim .env

# 3. 启动所有服务
docker-compose up -d

# 4. 查看服务状态
docker-compose ps

# 5. 查看日志
docker-compose logs -f api-gateway

# 6. 停止服务
docker-compose down
```

### 服务访问地址

| 服务 | 地址 | 说明 |
|------|------|------|
| 前端页面 | http://localhost | Nginx反向代理，已开启Gzip |
| API文档 | http://localhost:8000/docs | FastAPI Swagger UI |
| InfluxDB UI | http://localhost:8086 | 用户名/密码见.env |
| MQTT Broker | http://localhost:1883 | 匿名访问 |
| Redis | localhost:6379 | 密码见.env |

### 健康检查

```bash
# 检查所有服务健康状态
curl http://localhost:8000/health
curl http://localhost:8001/health
curl http://localhost:8002/health
curl http://localhost:8003/health
curl http://localhost:8004/health
curl http://localhost/health
```

---

## 传感器模拟器使用

### 增强版模拟器特性

- **7种预置场景**：直行巡航、平缓弯道、急弯行驶、湿滑路面、越野路况、高速行驶、古代驿道
- **6种路面条件**：干燥柏油、湿滑柏油、碎石路、泥泞路、冰雪路、古代驿道
- **自定义转弯半径**：支持固定转弯半径设置（正值右转，负值左转）
- **多车辆模拟**：支持同时模拟多辆双辕车
- **随机路面变化**：每10-20个周期随机切换路面条件

### 命令行参数

```bash
# 列出所有可用场景
python scripts/sensor_simulator_enhanced.py --list-scenarios

# 列出所有可用路面条件
python scripts/sensor_simulator_enhanced.py --list-roads

# 基础用法（默认场景：古代驿道）
python scripts/sensor_simulator_enhanced.py \
  --api-host localhost \
  --api-port 8000 \
  --interval 60 \
  --vehicles 3

# 指定场景：急弯行驶
python scripts/sensor_simulator_enhanced.py --scenario sharp_turns

# 指定路面条件：湿滑路面
python scripts/sensor_simulator_enhanced.py --road wet_asphalt

# 固定转弯半径（10米右转）
python scripts/sensor_simulator_enhanced.py --turning-radius 10.0

# 固定转弯半径（15米左转）
python scripts/sensor_simulator_enhanced.py --turning-radius -15.0

# 固定车速：3m/s
python scripts/sensor_simulator_enhanced.py --speed 3.0

# 组合参数：急弯 + 泥泞路 + 固定半径 + 多车辆
python scripts/sensor_simulator_enhanced.py \
  --scenario sharp_turns \
  --road muddy \
  --turning-radius 8.0 \
  --vehicles 5 \
  --interval 30
```

### 场景说明

| 场景ID | 名称 | 转弯半径 | 速度范围 | 典型路面 | 说明 |
|--------|------|----------|----------|----------|------|
| `straight_cruise` | 直行巡航 | 100-500m | 3-7m/s | 干燥柏油 | 平稳直行，辕杆小幅度修正 |
| `gentle_turns` | 平缓弯道 | 20-50m | 4-6m/s | 干燥柏油 | 大半径弯道，侧倾角小 |
| `sharp_turns` | 急弯行驶 | 5-15m | 2-4m/s | 古代驿道 | 小半径弯道，侧倾风险高 |
| `wet_road` | 湿滑路面 | 15-40m | 2.5-4.5m/s | 湿滑柏油 | 湿滑路面，滑移率升高 |
| `off_road` | 越野路况 | 10-30m | 1.5-3.5m/s | 泥泞路 | 泥泞越野，极易侧翻 |
| `highway` | 高速行驶 | 50-100m | 8-12m/s | 干燥柏油 | 高速行驶，横摆风险高 |
| `ancient_road` | 古代驿道 | 8-25m | 2-4m/s | 古代驿道 | 复原秦汉时期典型路况 |

### 路面条件说明

| 路面ID | 名称 | 摩擦系数 | 滑移因子 | 说明 |
|--------|------|----------|----------|------|
| `dry_asphalt` | 干燥柏油路 | 0.7-0.9 | 0.8 | 理想路况，抓地力强 |
| `wet_asphalt` | 湿滑柏油路 | 0.4-0.6 | 1.5 | 雨后路面，制动距离增加 |
| `gravel` | 碎石路 | 0.5-0.7 | 1.8 | 不平整路面，颠簸明显 |
| `muddy` | 泥泞路 | 0.2-0.4 | 2.5 | 泥泞土路，极易打滑 |
| `icy` | 冰雪路 | 0.1-0.25 | 3.0 | 冰雪覆盖，极度危险 |
| `dirt_track` | 古代驿道 | 0.35-0.55 | 1.6 | 秦汉时期典型土路 |

### Docker中运行模拟器

```bash
# 使用docker-compose启动的默认模拟器
docker-compose up -d sensor-simulator

# 自定义模拟器参数（修改.env）
SIMULATOR_INTERVAL=30
SIMULATOR_VEHICLE_COUNT=5

# 或直接运行容器
docker run -it --rm \
  --network chariot-simulation_chariot-net \
  -v $(pwd)/scripts:/app/scripts:ro \
  chariot-simulation-sensor-simulator \
  python scripts/sensor_simulator_enhanced.py \
  --api-host api-gateway \
  --scenario sharp_turns \
  --road muddy \
  --turning-radius 8.0
```

---

## API 接口

### REST API

```bash
# 发送传感器数据
curl -X POST http://localhost:8000/api/sensor/data \
  -H "Content-Type: application/json" \
  -d '{
    "vehicle_id": "chariot-001",
    "pole_angle": 15.5,
    "slip_rate": 0.12,
    "roll_angle": 8.2,
    "friction_coeff": 0.65
  }'

# 转向分析
curl -X POST http://localhost:8000/api/analysis/steering \
  -H "Content-Type: application/json" \
  -d '{
    "pole_angle": 20.0,
    "vehicle_speed": 5.0,
    "friction_coeff": 0.6
  }'

# 稳定性分析（支持载货配置）
curl -X POST http://localhost:8000/api/analysis/stability \
  -H "Content-Type: application/json" \
  -d '{
    "speed": 10.0,
    "pole_angle": 15.0,
    "roll_angle": 5.0,
    "slip_rate": 0.1,
    "friction_coeff": 0.6,
    "cargo_mass": 200.0,
    "cargo_offset_lateral": 0.1,
    "cargo_offset_height": 0.3
  }'

# 获取系统参数（几何/动力学/告警阈值）
curl http://localhost:8000/api/system/params

# 获取车辆最新数据
curl http://localhost:8000/api/data/latest/chariot-001

# 获取在线车辆列表
curl http://localhost:8000/api/vehicles
```

### WebSocket 实时推送

```javascript
const ws = new WebSocket('ws://localhost:8000/ws/realtime');

ws.onmessage = (event) => {
  const data = JSON.parse(event.data);
  switch (data.type) {
    case 'sensor_data':
      console.log('传感器数据:', data);
      break;
    case 'steering_result':
      console.log('转向结果:', data);
      break;
    case 'stability_result':
      console.log('稳定性结果:', data);
      break;
    case 'alert':
      console.log('告警:', data);
      break;
  }
};
```

### MQTT 订阅

```bash
# 订阅所有告警
mosquitto_sub -h localhost -t 'chariot/alerts/#' -v

# 订阅特定车辆的传感器数据
mosquitto_sub -h localhost -t 'chariot/data/chariot-001/#' -v

# 订阅特定告警类型
mosquitto_sub -h localhost -t 'chariot/alerts/+/roll_angle' -v
```

---

## 配置说明

### 外置JSON配置文件

| 文件 | 路径 | 说明 |
|------|------|------|
| 几何参数 | [config/json/chariot_geometry.json](config/json/chariot_geometry.json) | 轴距、轮距、阿克曼角、连杆参数 |
| 动力学参数 | [config/json/vehicle_dynamics.json](config/json/vehicle_dynamics.json) | 质量、转动惯量、侧倾刚度、阻尼比 |
| 系统配置 | [config/json/system_config.json](config/json/system_config.json) | Redis/MQTT/InfluxDB连接、通道定义 |
| 告警阈值 | [config/json/alert_thresholds.json](config/json/alert_thresholds.json) | 各告警类型阈值、冷却时间 |

### 环境变量配置

```bash
# Redis
REDIS_HOST=redis
REDIS_PORT=6379
REDIS_PASSWORD=chariot_redis_secret_2024

# InfluxDB
INFLUXDB_HOST=influxdb
INFLUXDB_PORT=8086
INFLUXDB_ORG=chariot-research
INFLUXDB_BUCKET=chariot_data
INFLUXDB_TOKEN=chariot_influx_secret_token_2024
INFLUXDB_ADMIN_USER=admin
INFLUXDB_ADMIN_PASSWORD=chariot_admin_2024

# MQTT
MQTT_HOST=mosquitto
MQTT_PORT=1883

# Gunicorn
GUNICORN_WORKERS=2
GUNICORN_TIMEOUT=120

# 模拟器
SIMULATOR_INTERVAL=60
SIMULATOR_VEHICLE_COUNT=3
SIMULATOR_TURNING_RADIUS_MIN=5.0
SIMULATOR_TURNING_RADIUS_MAX=50.0
```

---

## 生产级部署优化

### Gunicorn + Uvicorn 配置
- Worker进程数：CPU核数 × 2 + 1
- Worker类：`uvicorn.workers.UvicornWorker`
- 超时时间：120秒
- 最大请求数：1000（自动重启防止内存泄漏）
- 支持热重载（开发环境）

### Nginx 优化
- **Gzip压缩**：静态资源、JSON响应、字体图片等，压缩级别6
- **静态资源缓存**：设置1年缓存，immutable
- **WebSocket代理**：Connection升级，长连接超时24小时
- **安全头**：X-Frame-Options、X-XSS-Protection、Referrer-Policy

### InfluxDB 降采样
- 原始数据（1分钟级）保留7天
- Flux任务每小时聚合到1小时桶（保留7天）
- Flux任务每天聚合到1天桶（保留365天）

### 监控与告警
- Docker健康检查：每个服务30秒间隔
- Redis内存限制：256MB，LRU淘汰策略
- InfluxDB数据自动过期

---

## 目录结构

```
chariot-simulation/
├── backend/                     # 单体后端（保留兼容）
├── config/
│   ├── json/                    # 外置JSON配置
│   │   ├── chariot_geometry.json
│   │   ├── vehicle_dynamics.json
│   │   ├── system_config.json
│   │   └── alert_thresholds.json
│   └── settings.py              # 原全局配置（保留兼容）
├── docker/                      # Docker相关配置
│   ├── Dockerfile.python        # Python多阶段构建
│   ├── gunicorn_conf.py         # Gunicorn配置
│   ├── nginx.conf               # Nginx配置（含Gzip）
│   ├── mosquitto.conf           # MQTT Broker配置
│   └── init-influxdb.sh         # InfluxDB初始化（降采样）
├── frontend/                    # 前端代码
│   ├── index.html
│   ├── chariot_3d.js           # Three.js 3D渲染
│   ├── steering_panel.js       # 控制面板 + WebSocket
│   ├── app.js                  # 入口文件（保留兼容）
│   └── style.css
├── scripts/                     # 脚本
│   ├── init_influxdb.py         # 原InfluxDB初始化脚本
│   ├── sensor_simulator.py     # 原传感器模拟器
│   └── sensor_simulator_enhanced.py  # 增强版模拟器
├── services/                    # 微服务代码
│   ├── common/                  # 公共库
│   │   ├── config_loader.py     # JSON配置加载器
│   │   ├── redis_client.py      # Redis Pub/Sub封装
│   │   ├── message_protocol.py  # 消息协议定义
│   │   ├── steering_model.py    # 四杆机构+阿克曼模型
│   │   ├── stability_analysis.py  # 变重心动力学模型
│   │   └── alert_manager.py     # 告警管理器
│   ├── dtu_receiver/           # 传感器接收服务
│   ├── steering_simulator/      # 转向仿真服务
│   ├── stability_analyzer/     # 稳定性分析服务
│   ├── alarm_mqtt/             # 告警MQTT服务
│   ├── api_gateway/            # API网关服务
│   └── requirements.txt         # Python依赖
├── .env                         # 环境变量
├── .dockerignore                # Docker忽略文件
├── docker-compose.yml           # Docker编排
├── start_all.bat                # Windows一键启动脚本
├── _regression_test.py          # 回归测试脚本
└── README.md                    # 本文件
```

---

## 功能回归测试

```bash
# 运行完整回归测试（52项测试）
python _regression_test.py

# 测试内容
# [1/8] Config Loading Test        - JSON配置加载
# [2/8] Message Protocol Test     - 消息协议与校验
# [3/8] Steering Simulation Test   - 阿克曼+四杆机构
# [4/8] Stability Analysis Test    - 变重心+侧翻评估
# [5/8] Microservices Test         - 5个服务实例化
# [6/8] Redis Client Test          - 降级容错
# [7/8] Alert Manager Test         - 告警检测+冷却
# [8/8] Frontend Files Test        - 前端文件拆分
```

---

## 技术栈

### 后端
- **Python 3.11** - 主编程语言
- **FastAPI 0.104** - 高性能Web框架
- **Gunicorn + Uvicorn** - 生产级ASGI服务器
- **Redis 7** - 消息队列 + Pub/Sub通信
- **InfluxDB 2.7** - 时序数据库
- **Eclipse Mosquitto 2.0** - MQTT Broker
- **paho-mqtt 1.6** - MQTT客户端

### 前端
- **Three.js r160** - WebGL 3D渲染
- **GPU InstancedMesh** - 实例化渲染优化
- **Canvas 2D** - 连杆示意图
- **WebSocket** - 实时数据推送

### DevOps
- **Docker** - 容器化部署
- **Docker Compose** - 服务编排
- **Nginx 1.25** - 反向代理 + Gzip压缩
- **多阶段构建** - 镜像体积优化
- **健康检查** - 服务可用性监控

---

## 常见问题

### Q: 如何修改几何参数？
A: 编辑 [config/json/chariot_geometry.json](config/json/chariot_geometry.json)，重启服务即可生效，无需重新构建。

### Q: 如何调整告警阈值？
A: 编辑 [config/json/alert_thresholds.json](config/json/alert_thresholds.json)，或通过环境变量覆盖。

### Q: 模拟器数据如何导入生产环境？
A: 修改 `--api-host` 和 `--api-port` 指向生产环境API网关即可。

### Q: 如何扩展更多车辆？
A: 修改 `.env` 中的 `SIMULATOR_VEHICLE_COUNT`，或直接运行多个模拟器实例。

### Q: InfluxDB数据如何备份？
A: 使用 `influx backup` 命令，或挂载的 `influxdb-data` 卷直接复制。

---

## 版本历史

### v2.0.0 (2024)
- ✅ 微服务架构重构，5个独立服务通过Redis Pub/Sub通信
- ✅ 前端拆分为chariot_3d.js和steering_panel.js
- ✅ 几何/动力学参数外置为JSON配置
- ✅ 增强版传感器模拟器（场景/路面/转弯半径配置）
- ✅ Docker多阶段构建 + Compose编排
- ✅ Gunicorn+Uvicorn生产级部署
- ✅ InfluxDB降采样配置
- ✅ Nginx Gzip压缩
- ✅ 52项功能回归测试

### v1.1.0
- ✅ 四杆机构运动学修正（连杆干涉问题）
- ✅ 变重心动力学模型（货物移动影响）
- ✅ GPU实例化渲染优化

### v1.0.0
- ✅ 单体FastAPI后端 + InfluxDB存储
- ✅ 阿克曼转向几何计算
- ✅ 侧翻风险评估
- ✅ MQTT告警推送
- ✅ Three.js 3D前端

---

## 许可证

本项目仅供学术研究使用。

## 联系方式

交通史研究团队
