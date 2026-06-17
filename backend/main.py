import sys
import os
import time
from datetime import datetime, timedelta
from typing import Optional, List
from contextlib import asynccontextmanager

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
import numpy as np

from config.settings import (
    INFLUXDB_URL, INFLUXDB_TOKEN, INFLUXDB_ORG, INFLUXDB_BUCKET,
    MQTT_BROKER, MQTT_PORT, MQTT_TOPIC_ALERT, MQTT_TOPIC_DATA,
    FASTAPI_HOST, FASTAPI_PORT,
    ALERT_ROLL_ANGLE_THRESHOLD, ALERT_SLIP_RATE_LOW, ALERT_SLIP_RATE_HIGH,
    CHARIOT_WHEELBASE, CHARIOT_TRACK_WIDTH, CHARIOT_CG_HEIGHT, CHARIOT_ROLL_CENTER_HEIGHT
)
from steering_model import ChariotParams, MultiBodyDynamicsSteering
from stability_analysis import VehicleDynamicsParams, StabilityAnalyzer, CargoConfig
from alert_manager import AlertManager


try:
    from influxdb_client import InfluxDBClient, Point, WritePrecision
    from influxdb_client.client.write_api import SYNCHRONOUS
    INFLUXDB_AVAILABLE = True
except ImportError:
    INFLUXDB_AVAILABLE = False


influx_client = None
write_api = None
query_api = None
alert_manager = None
steering_model = None
stability_analyzer = None
websocket_connections: List[WebSocket] = []


class SensorData(BaseModel):
    vehicle_id: str
    pole_angle: float
    slip_rate: float
    roll_angle: float
    friction_coeff: float
    timestamp: Optional[int] = None


class SteeringRequest(BaseModel):
    pole_angle: float
    vehicle_speed: float = 5.0
    friction_coeff: float = 0.7
    duration: float = 10.0


class StabilityRequest(BaseModel):
    speed: float
    pole_angle: float
    roll_angle: float
    slip_rate: float = 0.1
    friction_coeff: float = 0.7
    cargo_mass: float = 0.0
    cargo_offset_lateral: float = 0.0
    cargo_offset_longitudinal: float = 0.0
    cargo_offset_height: float = 0.0


def init_influxdb():
    global influx_client, write_api, query_api
    if not INFLUXDB_AVAILABLE:
        print("警告: influxdb-client 未安装，InfluxDB功能不可用")
        return False
    try:
        influx_client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
        write_api = influx_client.write_api(write_options=SYNCHRONOUS)
        query_api = influx_client.query_api()
        print("InfluxDB 连接成功")
        return True
    except Exception as e:
        print(f"InfluxDB 连接失败: {e}")
        return False


def init_models():
    global steering_model, stability_analyzer
    chariot_params = ChariotParams(
        wheelbase=CHARIOT_WHEELBASE,
        track_width=CHARIOT_TRACK_WIDTH
    )
    dynamics_params = VehicleDynamicsParams(
        wheelbase=CHARIOT_WHEELBASE,
        track_width=CHARIOT_TRACK_WIDTH,
        cg_height=CHARIOT_CG_HEIGHT,
        roll_center_height=CHARIOT_ROLL_CENTER_HEIGHT
    )
    steering_model = MultiBodyDynamicsSteering(chariot_params)
    stability_analyzer = StabilityAnalyzer(dynamics_params)
    print("仿真模型初始化完成")


def init_alert_manager():
    global alert_manager
    alert_manager = AlertManager(
        broker=MQTT_BROKER,
        port=MQTT_PORT,
        topic_alert=MQTT_TOPIC_ALERT,
        topic_data=MQTT_TOPIC_DATA,
        roll_threshold=ALERT_ROLL_ANGLE_THRESHOLD,
        slip_low=ALERT_SLIP_RATE_LOW,
        slip_high=ALERT_SLIP_RATE_HIGH
    )
    connected = alert_manager.connect_mqtt()
    if not connected:
        print("提示: MQTT未连接，告警将只在本地记录")
    return alert_manager


@asynccontextmanager
async def lifespan(app: FastAPI):
    init_influxdb()
    init_models()
    init_alert_manager()
    yield
    if influx_client:
        influx_client.close()
    if alert_manager:
        alert_manager.close()


app = FastAPI(
    title="古代双辕车转向机构仿真与操控稳定性分析系统",
    description="基于阿克曼转向几何和多体动力学的双辕车仿真分析平台",
    version="1.0.0",
    lifespan=lifespan
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

frontend_dir = os.path.join(os.path.dirname(__file__), '..', 'frontend')
if os.path.exists(frontend_dir):
    app.mount("/frontend", StaticFiles(directory=frontend_dir, html=True), name="frontend")


def save_sensor_data(data: SensorData):
    if not write_api:
        return
    try:
        ts = data.timestamp if data.timestamp else int(time.time())
        point = Point("chariot_sensor") \
            .tag("vehicle_id", data.vehicle_id) \
            .field("pole_angle", data.pole_angle) \
            .field("slip_rate", data.slip_rate) \
            .field("roll_angle", data.roll_angle) \
            .field("friction_coeff", data.friction_coeff) \
            .time(ts, WritePrecision.S)
        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=point)
    except Exception as e:
        print(f"保存传感器数据失败: {e}")


def save_analysis_data(vehicle_id: str, steering_result: dict, stability_result: dict):
    if not write_api:
        return
    try:
        ts = int(time.time())
        steering_point = Point("steering_analysis") \
            .tag("vehicle_id", vehicle_id) \
            .field("turning_radius", steering_result.get("turning_radius", 0)) \
            .field("inner_wheel_angle", steering_result.get("inner_wheel_angle", 0)) \
            .field("outer_wheel_angle", steering_result.get("outer_wheel_angle", 0)) \
            .field("wheel_speed_diff", steering_result.get("wheel_speed_diff", 0)) \
            .field("ackermann_error", steering_result.get("ackermann_error", 0)) \
            .field("transmission_angle_inner", steering_result.get("transmission_angle_inner", 0)) \
            .field("transmission_angle_outer", steering_result.get("transmission_angle_outer", 0)) \
            .field("linkage_interference", int(steering_result.get("linkage_interference", False))) \
            .field("dead_point_risk", int(steering_result.get("dead_point_risk", False))) \
            .time(ts, WritePrecision.S)

        stability_point = Point("stability_analysis") \
            .tag("vehicle_id", vehicle_id) \
            .field("yaw_rate", stability_result.get("yaw_rate", 0)) \
            .field("roll_center_height", stability_result.get("roll_center_height", 0)) \
            .field("rollover_risk", stability_result.get("rollover_risk", 0)) \
            .field("lateral_acceleration", stability_result.get("lateral_acceleration", 0)) \
            .field("stability_index", stability_result.get("stability_index", 0)) \
            .field("critical_speed", stability_result.get("critical_speed", 0)) \
            .field("effective_cg_height", stability_result.get("effective_cg_height", 0)) \
            .field("effective_cg_lateral", stability_result.get("effective_cg_lateral", 0)) \
            .field("effective_cg_longitudinal", stability_result.get("effective_cg_longitudinal", 0)) \
            .field("effective_yaw_inertia", stability_result.get("effective_yaw_inertia", 0)) \
            .field("cargo_shift_lateral", stability_result.get("cargo_shift_lateral", 0)) \
            .field("cargo_shift_vertical", stability_result.get("cargo_shift_vertical", 0)) \
            .time(ts, WritePrecision.S)

        write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG,
                        record=[steering_point, stability_point])
    except Exception as e:
        print(f"保存分析数据失败: {e}")


async def broadcast_to_websockets(message: dict):
    for ws in websocket_connections:
        try:
            await ws.send_json(message)
        except:
            pass


@app.get("/")
async def root():
    return {
        "name": "古代双辕车转向机构仿真与操控稳定性分析系统",
        "version": "1.0.0",
        "endpoints": {
            "sensor_data": "/api/sensor/data",
            "steering_analysis": "/api/analysis/steering",
            "stability_analysis": "/api/analysis/stability",
            "wheel_trajectory": "/api/analysis/trajectory",
            "linkage_positions": "/api/analysis/linkage",
            "alerts": "/api/alerts",
            "historic_data": "/api/data/history"
        }
    }


@app.post("/api/sensor/data")
async def receive_sensor_data(data: SensorData):
    if data.timestamp is None:
        data.timestamp = int(time.time())

    save_sensor_data(data)

    steering_result = steering_model.calculate_steering(
        data.pole_angle, 5.0, data.friction_coeff
    )

    stability_result = stability_analyzer.analyze(
        speed=5.0,
        pole_angle_deg=data.pole_angle,
        roll_angle_deg=data.roll_angle,
        slip_rate=data.slip_rate,
        friction_coeff=data.friction_coeff
    )

    steering_dict = {
        "inner_wheel_angle": steering_result.inner_wheel_angle,
        "outer_wheel_angle": steering_result.outer_wheel_angle,
        "turning_radius": steering_result.turning_radius,
        "wheel_speed_diff": steering_result.wheel_speed_diff,
        "ackermann_error": steering_result.ackermann_error,
        "pole_effective_angle": steering_result.pole_effective_angle,
        "transmission_angle_inner": steering_result.transmission_angle_inner,
        "transmission_angle_outer": steering_result.transmission_angle_outer,
        "linkage_interference": steering_result.linkage_interference,
        "dead_point_risk": steering_result.dead_point_risk
    }

    stability_dict = {
        "roll_angle": stability_result.roll_angle,
        "yaw_rate": stability_result.yaw_rate,
        "lateral_acceleration": stability_result.lateral_acceleration,
        "roll_center_height": stability_result.roll_center_height,
        "rollover_risk": stability_result.rollover_risk,
        "stability_index": stability_result.stability_index,
        "understeer_gradient": stability_result.understeer_gradient,
        "critical_speed": stability_result.critical_speed,
        "effective_cg_height": stability_result.effective_cg_height,
        "effective_cg_lateral": stability_result.effective_cg_lateral,
        "effective_cg_longitudinal": stability_result.effective_cg_longitudinal,
        "effective_yaw_inertia": stability_result.effective_yaw_inertia,
        "cargo_shift_lateral": stability_result.cargo_shift_lateral,
        "cargo_shift_vertical": stability_result.cargo_shift_vertical
    }

    save_analysis_data(data.vehicle_id, steering_dict, stability_dict)

    alerts = alert_manager.check_sensor_data(
        data.vehicle_id,
        data.pole_angle,
        data.slip_rate,
        data.roll_angle,
        data.friction_coeff
    )

    alert_dicts = [
        {
            "vehicle_id": a.vehicle_id,
            "alert_type": a.alert_type,
            "severity": a.severity,
            "value": a.value,
            "threshold": a.threshold,
            "message": a.message,
            "timestamp": a.timestamp
        }
        for a in alerts
    ]

    result = {
        "status": "received",
        "timestamp": data.timestamp,
        "sensor_data": data.model_dump(),
        "steering_analysis": steering_dict,
        "stability_analysis": stability_dict,
        "alerts": alert_dicts
    }

    await broadcast_to_websockets(result)

    if alert_manager.mqtt_client:
        alert_manager.publish_data(data.vehicle_id, result)

    return result


@app.post("/api/analysis/steering")
async def analyze_steering(request: SteeringRequest):
    result = steering_model.calculate_steering(
        request.pole_angle, request.vehicle_speed, request.friction_coeff
    )
    return {
        "pole_angle": request.pole_angle,
        "vehicle_speed": request.vehicle_speed,
        "inner_wheel_angle": result.inner_wheel_angle,
        "outer_wheel_angle": result.outer_wheel_angle,
        "turning_radius": result.turning_radius,
        "wheel_speed_diff": result.wheel_speed_diff,
        "ackermann_error": result.ackermann_error,
        "pole_effective_angle": result.pole_effective_angle,
        "inner_wheel_speed_factor": 1 - result.wheel_speed_diff / 2,
        "outer_wheel_speed_factor": 1 + result.wheel_speed_diff / 2,
        "transmission_angle_inner": result.transmission_angle_inner,
        "transmission_angle_outer": result.transmission_angle_outer,
        "linkage_interference": result.linkage_interference,
        "dead_point_risk": result.dead_point_risk
    }


@app.post("/api/analysis/stability")
async def analyze_stability(request: StabilityRequest):
    if request.cargo_mass > 0 or request.cargo_offset_lateral != 0 or \
       request.cargo_offset_longitudinal != 0 or request.cargo_offset_height != 0:
        cargo_cfg = CargoConfig(
            mass=request.cargo_mass,
            offset_lateral=request.cargo_offset_lateral,
            offset_longitudinal=request.cargo_offset_longitudinal,
            offset_height=request.cargo_offset_height
        )
        stability_analyzer.set_cargo(cargo_cfg)
    else:
        stability_analyzer.set_cargo(CargoConfig())

    result = stability_analyzer.analyze(
        speed=request.speed,
        pole_angle_deg=request.pole_angle,
        roll_angle_deg=request.roll_angle,
        slip_rate=request.slip_rate,
        friction_coeff=request.friction_coeff
    )

    margin = stability_analyzer.calculate_stability_margin(
        speed=request.speed,
        pole_angle_deg=request.pole_angle,
        friction_coeff=request.friction_coeff
    )

    return {
        "roll_angle": result.roll_angle,
        "roll_rate": result.roll_rate,
        "yaw_rate": result.yaw_rate,
        "lateral_acceleration": result.lateral_acceleration,
        "roll_center_height": result.roll_center_height,
        "rollover_risk": result.rollover_risk,
        "stability_index": result.stability_index,
        "understeer_gradient": result.understeer_gradient,
        "critical_speed": result.critical_speed,
        "stability_margin": margin,
        "effective_cg_height": result.effective_cg_height,
        "effective_cg_lateral": result.effective_cg_lateral,
        "effective_cg_longitudinal": result.effective_cg_longitudinal,
        "effective_yaw_inertia": result.effective_yaw_inertia,
        "cargo_shift_lateral": result.cargo_shift_lateral,
        "cargo_shift_vertical": result.cargo_shift_vertical
    }


@app.post("/api/analysis/trajectory")
async def get_wheel_trajectory(request: SteeringRequest):
    trajectory = steering_model.get_wheel_trajectory(
        request.pole_angle, request.vehicle_speed, request.duration
    )
    return trajectory


@app.get("/api/analysis/linkage")
async def get_linkage_positions(pole_angle: float = Query(0.0, ge=-45, le=45)):
    positions = steering_model.get_linkage_positions(pole_angle)
    return positions


@app.get("/api/alerts")
async def get_alerts(vehicle_id: Optional[str] = None, limit: int = 50):
    alerts = alert_manager.get_recent_alerts(vehicle_id=vehicle_id, limit=limit)
    return {"alerts": alerts, "total": len(alerts)}


@app.get("/api/alerts/stats")
async def get_alert_stats():
    stats = alert_manager.get_alert_stats()
    return stats


@app.get("/api/data/history")
async def get_historic_data(
    vehicle_id: str,
    hours: int = Query(1, ge=1, le=720),
    measurement: str = "chariot_sensor"
):
    if not query_api:
        return {"data": [], "message": "InfluxDB不可用"}

    try:
        stop_time = datetime.utcnow()
        start_time = stop_time - timedelta(hours=hours)

        query = f'''
        from(bucket: "{INFLUXDB_BUCKET}")
        |> range(start: {start_time.isoformat()}Z, stop: {stop_time.isoformat()}Z)
        |> filter(fn: (r) => r["_measurement"] == "{measurement}")
        |> filter(fn: (r) => r["vehicle_id"] == "{vehicle_id}")
        |> sort(columns: ["_time"], desc: false)
        '''

        tables = query_api.query(query, org=INFLUXDB_ORG)

        data = {}
        for table in tables:
            for record in table.records:
                ts = record.get_time().timestamp()
                field = record.get_field()
                value = record.get_value()
                if ts not in data:
                    data[ts] = {"timestamp": ts}
                data[ts][field] = value

        data_list = sorted(data.values(), key=lambda x: x["timestamp"])
        return {"data": data_list, "count": len(data_list)}
    except Exception as e:
        return {"data": [], "error": str(e)}


@app.get("/api/vehicles")
async def get_vehicles():
    if not query_api:
        return {"vehicles": ["chariot-qin-001", "chariot-han-002", "chariot-zhou-003"]}

    try:
        query = f'''
        import "influxdata/influxdb/schema"
        schema.tagValues(bucket: "{INFLUXDB_BUCKET}", tag: "vehicle_id")
        '''
        tables = query_api.query(query, org=INFLUXDB_ORG)
        vehicles = []
        for table in tables:
            for record in table.records:
                vehicles.append(record.get_value())
        return {"vehicles": list(set(vehicles))}
    except Exception as e:
        return {"vehicles": ["chariot-qin-001", "chariot-han-002", "chariot-zhou-003"],
                "error": str(e)}


@app.get("/api/system/params")
async def get_system_params():
    return {
        "chariot": {
            "wheelbase": CHARIOT_WHEELBASE,
            "track_width": CHARIOT_TRACK_WIDTH,
            "cg_height": CHARIOT_CG_HEIGHT,
            "roll_center_height": CHARIOT_ROLL_CENTER_HEIGHT
        },
        "alerts": {
            "roll_angle_threshold": ALERT_ROLL_ANGLE_THRESHOLD,
            "slip_rate_low": ALERT_SLIP_RATE_LOW,
            "slip_rate_high": ALERT_SLIP_RATE_HIGH
        }
    }


@app.websocket("/ws/realtime")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    websocket_connections.append(websocket)
    try:
        while True:
            data = await websocket.receive_json()
    except WebSocketDisconnect:
        websocket_connections.remove(websocket)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=FASTAPI_HOST, port=FASTAPI_PORT)
