import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from typing import Dict, Any, List, Optional
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from pydantic import BaseModel
import uvicorn
import time
import uuid

from common import (
    get_config_loader, RedisClient, SensorData, SteeringResult,
    StabilityResult, Alert, timestamp,
    RoadSurfaceModel, MultiVehicleSteeringModel,
    ComparisonAnalyzer, VirtualDriveEngine, CargoConfig
)
from common.stability_analysis import CargoConfig as _CargoConfig


class SensorRequest(BaseModel):
    vehicle_id: str
    pole_angle: float
    slip_rate: float
    roll_angle: float
    friction_coeff: float
    timestamp: float = None


class SteeringRequest(BaseModel):
    pole_angle: float
    vehicle_speed: float = 5.0
    friction_coeff: float = 0.6


class StabilityRequest(BaseModel):
    speed: float
    pole_angle: float
    roll_angle: float
    slip_rate: float
    friction_coeff: float
    cargo_mass: float = 0.0
    cargo_offset_lateral: float = 0.0
    cargo_offset_longitudinal: float = 0.0
    cargo_offset_height: float = 0.0


class VehicleComparisonRequest(BaseModel):
    vehicle_types: List[str] = ["chariot_double", "wheelbarrow_single", "chariot_four_wheel", "modern_car"]
    pole_angle_deg: float = 15.0
    speed_mps: float = 5.0
    friction_coeff: float = 0.5
    road_type: str = "ancient_post_road"
    cargo_mass: float = 0.0
    cargo_offset_lateral: float = 0.0
    cargo_offset_longitudinal: float = 0.0
    cargo_offset_height: float = 0.0


class RoadComparisonRequest(BaseModel):
    vehicle_type: str = "chariot_double"
    pole_angle_deg: float = 15.0
    speed_mps: float = 5.0
    road_types: List[str] = ["stone_pavement", "dirt_road", "mud_road", "asphalt_dry", "ancient_post_road"]
    cargo_mass: float = 0.0
    cargo_offset_lateral: float = 0.0
    cargo_offset_longitudinal: float = 0.0
    cargo_offset_height: float = 0.0


class VirtualDriveStepRequest(BaseModel):
    session_id: str = None
    vehicle_type: str = "chariot_double"
    road_type: str = "ancient_post_road"
    pole_angle_deg: float = 0.0
    throttle: float = 0.0
    brake: float = 0.0
    cargo_mass: float = 0.0
    cargo_offset_lateral: float = 0.0
    cargo_offset_longitudinal: float = 0.0
    cargo_offset_height: float = 0.0
    dt: float = 0.05


config = get_config_loader()
app = FastAPI(title="API Gateway", version="2.0.0")

redis: RedisClient = None
channels: Dict[str, str] = {}

active_websockets: List[WebSocket] = []
latest_data: Dict[str, Dict[str, Any]] = {}

_road_model: RoadSurfaceModel = None
_vehicle_model: MultiVehicleSteeringModel = None
_comparison_analyzer: ComparisonAnalyzer = None
_virtual_drive_engine: VirtualDriveEngine = None


@app.on_event("startup")
async def startup():
    global redis, channels, _road_model, _vehicle_model, _comparison_analyzer, _virtual_drive_engine

    sys_cfg = config.system_config()
    channels = sys_cfg['redis_channels']

    redis = RedisClient(**sys_cfg['redis'])
    redis.connect()

    _road_model = RoadSurfaceModel()
    _vehicle_model = MultiVehicleSteeringModel()
    _comparison_analyzer = ComparisonAnalyzer()
    _virtual_drive_engine = VirtualDriveEngine()

    redis.subscribe(channels['sensor_data_validated'], broadcast_ws)
    redis.subscribe(channels['steering_result'], broadcast_ws)
    redis.subscribe(channels['stability_result'], broadcast_ws)
    redis.subscribe(channels['alerts'], broadcast_ws)
    redis.subscribe(channels.get('comparison_result', 'chariot:result:comparison'), broadcast_ws)
    redis.subscribe(channels.get('virtual_drive', 'chariot:virtual:drive'), broadcast_ws)

    print(f"[api_gateway] 启动完成, HTTP端口: 8000")


def broadcast_ws(data: Dict[str, Any]) -> None:
    if 'type' not in data:
        if 'linkage_interference' in data:
            data['type'] = 'steering_result'
        elif 'effective_cg_height' in data:
            data['type'] = 'stability_result'
        elif 'alert_type' in data:
            data['type'] = 'alert'
        elif 'slip_rate' in data:
            data['type'] = 'sensor_data'

    # 缓存最新数据
    vid = data.get('vehicle_id', 'unknown')
    dtype = data.get('type', 'unknown')
    if vid not in latest_data:
        latest_data[vid] = {}
    latest_data[vid][dtype] = data

    # 广播
    disconnected = []
    for ws in active_websockets:
        try:
            import json
            ws.send_json(data)
        except Exception:
            disconnected.append(ws)

    for ws in disconnected:
        if ws in active_websockets:
            active_websockets.remove(ws)


@app.websocket("/ws/realtime")
async def websocket_realtime(websocket: WebSocket):
    await websocket.accept()
    active_websockets.append(websocket)
    try:
        # 发送当前系统参数
        params = {
            "type": "system_params",
            "chariot_geometry": config.chariot_geometry(),
            "vehicle_dynamics": config.vehicle_dynamics(),
            "alert_thresholds": config.alert_thresholds(),
            "vehicle_types": _vehicle_model.list_vehicle_types() if _vehicle_model else [],
            "road_surfaces": _road_model.list_road_types() if _road_model else []
        }
        await websocket.send_json(params)

        # 最新数据快照
        if latest_data:
            await websocket.send_json({
                "type": "snapshot",
                "vehicles": list(latest_data.keys()),
                "latest": latest_data
            })

        while True:
            msg = await websocket.receive_json()
            if msg.get('type') == 'ping':
                await websocket.send_json({"type": "pong", "timestamp": time.time()})
    except WebSocketDisconnect:
        if websocket in active_websockets:
            active_websockets.remove(websocket)


@app.post("/api/sensor/data")
async def receive_sensor_data(request: SensorRequest):
    if request.timestamp is None:
        request.timestamp = time.time()

    # 校验
    sensor = SensorData(
        vehicle_id=request.vehicle_id,
        pole_angle=request.pole_angle,
        slip_rate=request.slip_rate,
        roll_angle=request.roll_angle,
        friction_coeff=request.friction_coeff,
        timestamp=request.timestamp
    )
    if not sensor.validate():
        raise HTTPException(status_code=400, detail="数据校验失败")

    # 发布给DTU通道（dtu_receiver作为独立服务也可接收，但网关也转发）
    redis.publish(channels['sensor_data_raw'], sensor.to_dict())
    redis.publish(channels['sensor_data_validated'], sensor.to_dict())

    return {"status": "ok", "timestamp": sensor.timestamp, "vehicle_id": sensor.vehicle_id}


@app.post("/api/analysis/steering")
async def analyze_steering(req: SteeringRequest):
    sys_cfg = config.system_config()
    resp = redis.request_response(
        request_channel=channels['api_request']['steering'],
        response_channel=channels['api_response']['steering'],
        request=req.dict()
    )
    if resp is None:
        raise HTTPException(status_code=504, detail="转向仿真服务超时或不可用")
    if not resp.get('success'):
        raise HTTPException(status_code=500, detail=resp.get('error', '未知错误'))
    return {
        "pole_angle": req.pole_angle,
        "vehicle_speed": req.vehicle_speed,
        **resp['data']
    }


@app.post("/api/analysis/stability")
async def analyze_stability(req: StabilityRequest):
    sys_cfg = config.system_config()
    resp = redis.request_response(
        request_channel=channels['api_request']['stability'],
        response_channel=channels['api_response']['stability'],
        request=req.dict()
    )
    if resp is None:
        raise HTTPException(status_code=504, detail="稳定性分析服务超时或不可用")
    if not resp.get('success'):
        raise HTTPException(status_code=500, detail=resp.get('error', '未知错误'))
    return {
        "roll_angle": req.roll_angle,
        "speed": req.speed,
        "pole_angle": req.pole_angle,
        **resp['data']
    }


@app.get("/api/system/params")
async def get_system_params():
    return {
        "chariot_geometry": config.chariot_geometry(),
        "vehicle_dynamics": config.vehicle_dynamics(),
        "alert_thresholds": config.alert_thresholds(),
        "vehicle_types": _vehicle_model.list_vehicle_types() if _vehicle_model else [],
        "road_surfaces": _road_model.list_road_types() if _road_model else []
    }


@app.get("/api/vehicle/types")
async def list_vehicle_types():
    if not _vehicle_model:
        raise HTTPException(status_code=503, detail="服务未就绪")
    return {"vehicles": _vehicle_model.list_vehicle_types()}


@app.get("/api/road/surfaces")
async def list_road_surfaces():
    if not _road_model:
        raise HTTPException(status_code=503, detail="服务未就绪")
    return {"surfaces": _road_model.list_road_types()}


@app.post("/api/comparison/vehicles")
async def compare_vehicles(req: VehicleComparisonRequest):
    if not _comparison_analyzer:
        raise HTTPException(status_code=503, detail="服务未就绪")
    cargo = _CargoConfig(
        mass=req.cargo_mass,
        offset_lateral=req.cargo_offset_lateral,
        offset_longitudinal=req.cargo_offset_longitudinal,
        offset_height=req.cargo_offset_height
    )
    result = _comparison_analyzer.compare_vehicles(
        vehicle_types=req.vehicle_types,
        pole_angle_deg=req.pole_angle_deg,
        speed_mps=req.speed_mps,
        friction_coeff=req.friction_coeff,
        road_type=req.road_type,
        cargo=cargo
    )
    return result.to_dict()


@app.post("/api/comparison/roads")
async def compare_roads(req: RoadComparisonRequest):
    if not _comparison_analyzer:
        raise HTTPException(status_code=503, detail="服务未就绪")
    cargo = _CargoConfig(
        mass=req.cargo_mass,
        offset_lateral=req.cargo_offset_lateral,
        offset_longitudinal=req.cargo_offset_longitudinal,
        offset_height=req.cargo_offset_height
    )
    result = _comparison_analyzer.compare_road_surfaces(
        vehicle_type=req.vehicle_type,
        pole_angle_deg=req.pole_angle_deg,
        speed_mps=req.speed_mps,
        road_types=req.road_types,
        cargo=cargo
    )
    return result.to_dict()


@app.post("/api/virtual-drive/step")
async def virtual_drive_step(req: VirtualDriveStepRequest):
    if not _virtual_drive_engine:
        raise HTTPException(status_code=503, detail="服务未就绪")
    if req.session_id is None:
        req.session_id = str(uuid.uuid4())
    cargo = _CargoConfig(
        mass=req.cargo_mass,
        offset_lateral=req.cargo_offset_lateral,
        offset_longitudinal=req.cargo_offset_longitudinal,
        offset_height=req.cargo_offset_height
    )
    state = _virtual_drive_engine.step(
        session_id=req.session_id,
        vehicle_type=req.vehicle_type,
        road_type=req.road_type,
        pole_angle_deg=req.pole_angle_deg,
        throttle=req.throttle,
        brake=req.brake,
        cargo=cargo,
        dt=req.dt
    )
    return state.to_dict()


@app.post("/api/virtual-drive/reset/{session_id}")
async def virtual_drive_reset(session_id: str):
    if not _virtual_drive_engine:
        raise HTTPException(status_code=503, detail="服务未就绪")
    _virtual_drive_engine.reset_session(session_id)
    return {"status": "ok", "session_id": session_id}


@app.get("/api/data/latest/{vehicle_id}")
async def get_latest_data(vehicle_id: str):
    return latest_data.get(vehicle_id, {})


@app.get("/api/vehicles")
async def list_vehicles():
    return {"vehicles": list(latest_data.keys())}


@app.get("/health")
async def health_check():
    return {
        "status": "ok",
        "service": "api_gateway",
        "connected_websockets": len(active_websockets),
        "redis_connected": redis.client is not None
    }


if __name__ == "__main__":
    sys_cfg = config.system_config()
    uvicorn.run(app, host=sys_cfg['fastapi']['host'], port=sys_cfg['fastapi']['port'])
