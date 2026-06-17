from dataclasses import dataclass, asdict, field
from typing import Optional, Dict, Any
import time


def timestamp() -> float:
    return time.time()


@dataclass
class SensorData:
    vehicle_id: str
    pole_angle: float
    slip_rate: float
    roll_angle: float
    friction_coeff: float
    timestamp: float = field(default_factory=timestamp)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'SensorData':
        return cls(
            vehicle_id=data['vehicle_id'],
            pole_angle=float(data['pole_angle']),
            slip_rate=float(data['slip_rate']),
            roll_angle=float(data['roll_angle']),
            friction_coeff=float(data['friction_coeff']),
            timestamp=float(data.get('timestamp', timestamp()))
        )

    def validate(self) -> bool:
        if not self.vehicle_id or len(self.vehicle_id) < 3:
            return False
        if not (-60 <= self.pole_angle <= 60):
            return False
        if not (0 <= self.slip_rate <= 1):
            return False
        if not (-90 <= self.roll_angle <= 90):
            return False
        if not (0.1 <= self.friction_coeff <= 1.0):
            return False
        return True


@dataclass
class SteeringResult:
    inner_wheel_angle: float
    outer_wheel_angle: float
    turning_radius: float
    wheel_speed_diff: float
    ackermann_error: float
    pole_effective_angle: float
    transmission_angle_inner: float
    transmission_angle_outer: float
    linkage_interference: bool
    dead_point_risk: bool
    vehicle_id: str
    pole_angle_input: float
    vehicle_speed: float
    friction_coeff: float
    timestamp: float = field(default_factory=timestamp)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'SteeringResult':
        return cls(
            inner_wheel_angle=float(data['inner_wheel_angle']),
            outer_wheel_angle=float(data['outer_wheel_angle']),
            turning_radius=float(data['turning_radius']),
            wheel_speed_diff=float(data['wheel_speed_diff']),
            ackermann_error=float(data['ackermann_error']),
            pole_effective_angle=float(data['pole_effective_angle']),
            transmission_angle_inner=float(data['transmission_angle_inner']),
            transmission_angle_outer=float(data['transmission_angle_outer']),
            linkage_interference=bool(data['linkage_interference']),
            dead_point_risk=bool(data['dead_point_risk']),
            vehicle_id=data['vehicle_id'],
            pole_angle_input=float(data['pole_angle_input']),
            vehicle_speed=float(data['vehicle_speed']),
            friction_coeff=float(data['friction_coeff']),
            timestamp=float(data.get('timestamp', timestamp()))
        )


@dataclass
class StabilityResult:
    roll_angle: float
    roll_rate: float
    yaw_rate: float
    lateral_acceleration: float
    roll_center_height: float
    rollover_risk: float
    stability_index: float
    understeer_gradient: float
    critical_speed: float
    effective_cg_height: float
    effective_cg_lateral: float
    effective_cg_longitudinal: float
    effective_yaw_inertia: float
    cargo_shift_lateral: float
    cargo_shift_vertical: float
    vehicle_id: str
    pole_angle_input: float
    speed: float
    cargo_mass: float
    cargo_offset_lateral: float
    cargo_offset_longitudinal: float
    cargo_offset_height: float
    timestamp: float = field(default_factory=timestamp)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'StabilityResult':
        return cls(
            roll_angle=float(data['roll_angle']),
            roll_rate=float(data['roll_rate']),
            yaw_rate=float(data['yaw_rate']),
            lateral_acceleration=float(data['lateral_acceleration']),
            roll_center_height=float(data['roll_center_height']),
            rollover_risk=float(data['rollover_risk']),
            stability_index=float(data['stability_index']),
            understeer_gradient=float(data['understeer_gradient']),
            critical_speed=float(data['critical_speed']),
            effective_cg_height=float(data['effective_cg_height']),
            effective_cg_lateral=float(data['effective_cg_lateral']),
            effective_cg_longitudinal=float(data['effective_cg_longitudinal']),
            effective_yaw_inertia=float(data['effective_yaw_inertia']),
            cargo_shift_lateral=float(data['cargo_shift_lateral']),
            cargo_shift_vertical=float(data['cargo_shift_vertical']),
            vehicle_id=data['vehicle_id'],
            pole_angle_input=float(data['pole_angle_input']),
            speed=float(data['speed']),
            cargo_mass=float(data.get('cargo_mass', 0)),
            cargo_offset_lateral=float(data.get('cargo_offset_lateral', 0)),
            cargo_offset_longitudinal=float(data.get('cargo_offset_longitudinal', 0)),
            cargo_offset_height=float(data.get('cargo_offset_height', 0)),
            timestamp=float(data.get('timestamp', timestamp()))
        )


@dataclass
class Alert:
    vehicle_id: str
    alert_type: str
    severity: str
    value: float
    threshold: float
    message: str
    timestamp: float = field(default_factory=timestamp)
    acknowledged: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'Alert':
        return cls(
            vehicle_id=data['vehicle_id'],
            alert_type=data['alert_type'],
            severity=data['severity'],
            value=float(data['value']),
            threshold=float(data['threshold']),
            message=data['message'],
            timestamp=float(data.get('timestamp', timestamp())),
            acknowledged=bool(data.get('acknowledged', False))
        )


def create_response(request_id: str, success: bool, data: Dict[str, Any] = None,
                    error: str = None) -> Dict[str, Any]:
    return {
        'request_id': request_id,
        'success': success,
        'data': data or {},
        'error': error
    }
