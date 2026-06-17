import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from typing import Dict, Any
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn

from common import (
    get_config_loader, RedisClient, SensorData, SteeringResult
)
from common.steering_model import ChariotParams, MultiBodyDynamicsSteering


class SteeringRequest(BaseModel):
    pole_angle: float
    vehicle_speed: float = 5.0
    friction_coeff: float = 0.6
    vehicle_id: str = "manual"


config = get_config_loader()
app = FastAPI(title="Steering Simulator Service", version="2.0.0")

redis: RedisClient = None
steering_model: MultiBodyDynamicsSteering = None
channels: Dict[str, str] = {}


@app.on_event("startup")
async def startup():
    global redis, steering_model, channels

    sys_cfg = config.system_config()
    channels = sys_cfg['redis_channels']

    redis = RedisClient(**sys_cfg['redis'])
    redis.connect()

    geo_params = config.chariot_geometry()
    cp = ChariotParams(**geo_params)
    steering_model = MultiBodyDynamicsSteering(cp)

    redis.subscribe(channels['sensor_data_validated'], on_sensor_data)
    redis.subscribe(channels['api_request']['steering'], on_api_request)

    print(f"[steering_simulator] 启动完成, 订阅: {channels['sensor_data_validated']}")


def on_sensor_data(data: Dict[str, Any]) -> None:
    try:
        sensor = SensorData.from_dict(data)
        result = steering_model.calculate_steering(
            pole_angle=sensor.pole_angle,
            vehicle_speed=5.0,
            friction_coeff=sensor.friction_coeff
        )

        steering_result = SteeringResult(
            inner_wheel_angle=result.inner_wheel_angle,
            outer_wheel_angle=result.outer_wheel_angle,
            turning_radius=result.turning_radius,
            wheel_speed_diff=result.wheel_speed_diff,
            ackermann_error=result.ackermann_error,
            pole_effective_angle=result.pole_effective_angle,
            transmission_angle_inner=result.transmission_angle_inner,
            transmission_angle_outer=result.transmission_angle_outer,
            linkage_interference=result.linkage_interference,
            dead_point_risk=result.dead_point_risk,
            vehicle_id=sensor.vehicle_id,
            pole_angle_input=sensor.pole_angle,
            vehicle_speed=5.0,
            friction_coeff=sensor.friction_coeff,
            timestamp=sensor.timestamp
        )

        redis.publish(channels['steering_result'], steering_result.to_dict())
        print(f"[steering] 车辆={sensor.vehicle_id} 辕杆={sensor.pole_angle:.1f}° 内轮={result.inner_wheel_angle:.2f}° 死点={result.dead_point_risk}")

    except Exception as e:
        print(f"[steering] 处理传感器数据失败: {e}")


def on_api_request(data: Dict[str, Any]) -> None:
    try:
        request_id = data.get('request_id')
        req = SteeringRequest(**data)
        result = steering_model.calculate_steering(
            pole_angle=req.pole_angle,
            vehicle_speed=req.vehicle_speed,
            friction_coeff=req.friction_coeff
        )

        response = {
            'inner_wheel_angle': result.inner_wheel_angle,
            'outer_wheel_angle': result.outer_wheel_angle,
            'turning_radius': result.turning_radius,
            'wheel_speed_diff': result.wheel_speed_diff,
            'ackermann_error': result.ackermann_error,
            'pole_effective_angle': result.pole_effective_angle,
            'transmission_angle_inner': result.transmission_angle_inner,
            'transmission_angle_outer': result.transmission_angle_outer,
            'linkage_interference': result.linkage_interference,
            'dead_point_risk': result.dead_point_risk,
            'inner_wheel_speed_factor': 1 - result.wheel_speed_diff / 2,
            'outer_wheel_speed_factor': 1 + result.wheel_speed_diff / 2
        }

        from common import create_response
        redis.publish(
            channels['api_response']['steering'],
            create_response(request_id, True, response)
        )

    except Exception as e:
        from common import create_response
        request_id = data.get('request_id', '')
        redis.publish(
            channels['api_response']['steering'],
            create_response(request_id, False, error=str(e))
        )


@app.get("/health")
async def health_check():
    return {"status": "ok", "service": "steering_simulator"}


@app.post("/api/steering")
async def steering_analysis(req: SteeringRequest):
    result = steering_model.calculate_steering(
        pole_angle=req.pole_angle,
        vehicle_speed=req.vehicle_speed,
        friction_coeff=req.friction_coeff
    )
    return {
        "pole_angle": req.pole_angle,
        "vehicle_speed": req.vehicle_speed,
        "inner_wheel_angle": result.inner_wheel_angle,
        "outer_wheel_angle": result.outer_wheel_angle,
        "turning_radius": result.turning_radius,
        "transmission_angle_inner": result.transmission_angle_inner,
        "transmission_angle_outer": result.transmission_angle_outer,
        "linkage_interference": result.linkage_interference,
        "dead_point_risk": result.dead_point_risk
    }


if __name__ == "__main__":
    sys_cfg = config.system_config()
    uvicorn.run(app, host=sys_cfg['fastapi']['host'], port=8002)
