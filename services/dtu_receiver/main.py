import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from typing import Dict, Any
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn
import time

from common import get_config_loader, RedisClient, SensorData


class SensorRequest(BaseModel):
    vehicle_id: str
    pole_angle: float
    slip_rate: float
    roll_angle: float
    friction_coeff: float
    timestamp: float = None


config = get_config_loader()
app = FastAPI(title="DTU Receiver Service", version="2.0.0")

redis: RedisClient = None
channels: Dict[str, str] = {}


@app.on_event("startup")
async def startup():
    global redis, channels

    sys_cfg = config.system_config()
    channels = sys_cfg['redis_channels']

    redis = RedisClient(**sys_cfg['redis'])
    redis.connect()

    print(f"[dtu_receiver] 启动完成, 接收端口: 8001")


@app.post("/api/sensor/data")
async def receive_sensor_data(request: SensorRequest):
    if request.timestamp is None:
        request.timestamp = time.time()

    # 1. 发布原始数据（方便调试/回放）
    redis.publish(channels['sensor_data_raw'], request.dict())

    # 2. 校验数据
    sensor = SensorData(
        vehicle_id=request.vehicle_id,
        pole_angle=request.pole_angle,
        slip_rate=request.slip_rate,
        roll_angle=request.roll_angle,
        friction_coeff=request.friction_coeff,
        timestamp=request.timestamp
    )

    if not sensor.validate():
        raise HTTPException(
            status_code=400,
            detail=f"数据校验失败: 车辆={request.vehicle_id}, 辕杆={request.pole_angle}, 滑移={request.slip_rate}, 侧倾={request.roll_angle}, 摩擦={request.friction_coeff}"
        )

    # 3. 发布校验通过的数据
    redis.publish(channels['sensor_data_validated'], sensor.to_dict())

    print(f"[dtu] 接收: 车辆={sensor.vehicle_id} 辕杆={sensor.pole_angle:.1f}° 侧倾={sensor.roll_angle:.1f}° 滑移={sensor.slip_rate:.2f}")

    return {
        "status": "ok",
        "timestamp": sensor.timestamp,
        "vehicle_id": sensor.vehicle_id,
        "validated": True
    }


@app.get("/health")
async def health_check():
    return {"status": "ok", "service": "dtu_receiver"}


if __name__ == "__main__":
    sys_cfg = config.system_config()
    uvicorn.run(app, host=sys_cfg['fastapi']['host'], port=8001)
