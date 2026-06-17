import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from typing import Dict, Any, Optional
from fastapi import FastAPI
from pydantic import BaseModel
import uvicorn

from common import (
    get_config_loader, RedisClient, SteeringResult, StabilityResult, Alert,
    create_response
)
from common.stability_analysis import (
    VehicleDynamicsParams, StabilityAnalyzer, CargoConfig
)


class StabilityRequest(BaseModel):
    speed: float
    pole_angle: float
    roll_angle: float
    slip_rate: float
    friction_coeff: float
    vehicle_id: str = "manual"
    cargo_mass: float = 0.0
    cargo_offset_lateral: float = 0.0
    cargo_offset_longitudinal: float = 0.0
    cargo_offset_height: float = 0.0


config = get_config_loader()
app = FastAPI(title="Stability Analyzer Service", version="2.0.0")

redis: RedisClient = None
stability_analyzer: StabilityAnalyzer = None
channels: Dict[str, str] = {}

analyzer_cache: Dict[str, StabilityAnalyzer] = {}


@app.on_event("startup")
async def startup():
    global redis, channels

    sys_cfg = config.system_config()
    channels = sys_cfg['redis_channels']

    redis = RedisClient(**sys_cfg['redis'])
    redis.connect()

    redis.subscribe(channels['steering_result'], on_steering_result)
    redis.subscribe(channels['api_request']['stability'], on_api_request)

    print(f"[stability_analyzer] 启动完成, 订阅: {channels['steering_result']}")


def get_or_create_analyzer(vehicle_id: str) -> StabilityAnalyzer:
    if vehicle_id not in analyzer_cache:
        dyn_params = config.vehicle_dynamics()
        vp = VehicleDynamicsParams(**dyn_params)
        analyzer_cache[vehicle_id] = StabilityAnalyzer(vp)
    return analyzer_cache[vehicle_id]


def on_steering_result(data: Dict[str, Any]) -> None:
    try:
        steering = SteeringResult.from_dict(data)
        analyzer = get_or_create_analyzer(steering.vehicle_id)

        # 关联传感器原始数据需要额外缓存，这里简化从steering传
        # 假设传感器也在steering result中带了足够字段
        result = analyzer.analyze(
            speed=steering.vehicle_speed,
            pole_angle_deg=steering.pole_angle_input,
            roll_angle_deg=0.0,
            slip_rate=0.1,
            friction_coeff=steering.friction_coeff
        )

        margin = analyzer.calculate_stability_margin(
            speed=steering.vehicle_speed,
            pole_angle_deg=steering.pole_angle_input,
            friction_coeff=steering.friction_coeff
        )

        stability_result = StabilityResult(
            roll_angle=result.roll_angle,
            roll_rate=result.roll_rate,
            yaw_rate=result.yaw_rate,
            lateral_acceleration=result.lateral_acceleration,
            roll_center_height=result.roll_center_height,
            rollover_risk=result.rollover_risk,
            stability_index=result.stability_index,
            understeer_gradient=result.understeer_gradient,
            critical_speed=result.critical_speed,
            effective_cg_height=result.effective_cg_height,
            effective_cg_lateral=result.effective_cg_lateral,
            effective_cg_longitudinal=result.effective_cg_longitudinal,
            effective_yaw_inertia=result.effective_yaw_inertia,
            cargo_shift_lateral=result.cargo_shift_lateral,
            cargo_shift_vertical=result.cargo_shift_vertical,
            vehicle_id=steering.vehicle_id,
            pole_angle_input=steering.pole_angle_input,
            speed=steering.vehicle_speed,
            cargo_mass=0.0,
            cargo_offset_lateral=0.0,
            cargo_offset_longitudinal=0.0,
            cargo_offset_height=0.0,
            timestamp=steering.timestamp
        )

        redis.publish(channels['stability_result'], stability_result.to_dict())
        print(f"[stability] 车辆={steering.vehicle_id} 横摆={result.yaw_rate:.2f}°/s 侧翻={result.rollover_risk:.1f}% 稳定={result.stability_index:.2f}")

    except Exception as e:
        print(f"[stability] 处理转向结果失败: {e}")


def on_api_request(data: Dict[str, Any]) -> None:
    try:
        request_id = data.get('request_id')
        req = StabilityRequest(**data)
        analyzer = get_or_create_analyzer(req.vehicle_id)

        if req.cargo_mass > 0 or req.cargo_offset_lateral != 0 or \
           req.cargo_offset_longitudinal != 0 or req.cargo_offset_height != 0:
            cargo_cfg = CargoConfig(
                mass=req.cargo_mass,
                offset_lateral=req.cargo_offset_lateral,
                offset_longitudinal=req.cargo_offset_longitudinal,
                offset_height=req.cargo_offset_height
            )
            analyzer.set_cargo(cargo_cfg)
        else:
            analyzer.set_cargo(CargoConfig())

        result = analyzer.analyze(
            speed=req.speed,
            pole_angle_deg=req.pole_angle,
            roll_angle_deg=req.roll_angle,
            slip_rate=req.slip_rate,
            friction_coeff=req.friction_coeff
        )

        margin = analyzer.calculate_stability_margin(
            speed=req.speed,
            pole_angle_deg=req.pole_angle,
            friction_coeff=req.friction_coeff
        )

        response = {
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

        redis.publish(
            channels['api_response']['stability'],
            create_response(request_id, True, response)
        )

    except Exception as e:
        request_id = data.get('request_id', '')
        redis.publish(
            channels['api_response']['stability'],
            create_response(request_id, False, error=str(e))
        )


@app.get("/health")
async def health_check():
    return {"status": "ok", "service": "stability_analyzer"}


@app.post("/api/stability")
async def stability_analysis(req: StabilityRequest):
    analyzer = get_or_create_analyzer(req.vehicle_id)

    if req.cargo_mass > 0 or req.cargo_offset_lateral != 0 or \
       req.cargo_offset_longitudinal != 0 or req.cargo_offset_height != 0:
        cargo_cfg = CargoConfig(
            mass=req.cargo_mass,
            offset_lateral=req.cargo_offset_lateral,
            offset_longitudinal=req.cargo_offset_longitudinal,
            offset_height=req.cargo_offset_height
        )
        analyzer.set_cargo(cargo_cfg)

    result = analyzer.analyze(
        speed=req.speed,
        pole_angle_deg=req.pole_angle,
        roll_angle_deg=req.roll_angle,
        slip_rate=req.slip_rate,
        friction_coeff=req.friction_coeff
    )

    return {
        "yaw_rate": result.yaw_rate,
        "rollover_risk": result.rollover_risk,
        "stability_index": result.stability_index,
        "effective_cg_height": result.effective_cg_height,
        "effective_cg_lateral": result.effective_cg_lateral,
        "cargo_shift_lateral": result.cargo_shift_lateral
    }


if __name__ == "__main__":
    sys_cfg = config.system_config()
    uvicorn.run(app, host=sys_cfg['fastapi']['host'], port=8003)
