from dataclasses import dataclass, asdict, field
from typing import Optional, Dict, Any, List
from enum import Enum
import time


class VehicleType(str, Enum):
    CHARIOT_DOUBLE = "chariot_double"
    WHEELBARROW_SINGLE = "wheelbarrow_single"
    CHARIOT_FOUR_WHEEL = "chariot_four_wheel"
    MODERN_CAR = "modern_car"


class RoadSurface(str, Enum):
    ASPHALT_DRY = "asphalt_dry"
    STONE_PAVEMENT = "stone_pavement"
    DIRT_ROAD = "dirt_road"
    MUD_ROAD = "mud_road"
    GRAVEL_ROAD = "gravel_road"
    SAND_ROAD = "sand_road"
    ICE_SNOW = "ice_snow"
    ANCIENT_POST_ROAD = "ancient_post_road"


class SteeringMechanism(str, Enum):
    FOUR_BAR_ACKERMANN = "four_bar_ackermann"
    SINGLE_WHEEL_DIRECT = "single_wheel_direct"
    FRONT_AXLE_ACKERMANN = "front_axle_ackermann"
    RACK_PINION_ACKERMANN = "rack_pinion_ackermann"
    PURE_ACKERMANN = "pure_ackermann"


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


@dataclass
class RoadEffect:
    friction_coeff: float
    rolling_resistance: float
    slip_factor: float
    bump_amplitude: float
    irregularity: float
    road_type: str = ""
    effective_cornering_stiffness_front: float = 0.0
    effective_cornering_stiffness_rear: float = 0.0
    vibration_acceleration: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class VehicleComparisonEntry:
    vehicle_type: str
    vehicle_name: str
    era: str
    category: str
    steering_mechanism: str
    inner_wheel_angle: float
    outer_wheel_angle: float
    turning_radius: float
    ackermann_error: float
    max_inner_wheel_angle: float
    min_turning_radius: float
    transmission_angle_min: float
    yaw_rate: float
    lateral_acceleration: float
    rollover_risk: float
    stability_index: float
    critical_speed: float
    ssf_static: float
    understeer_gradient: float
    max_speed_mps: float
    mass: float
    cg_height: float
    wheelbase: float
    track_width: float
    propulsion: str

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class RoadComparisonEntry:
    road_type: str
    road_name: str
    category: str
    friction_coeff: float
    rolling_resistance: float
    slip_factor: float
    effective_speed: float
    turning_radius_effective: float
    yaw_rate: float
    lateral_acceleration: float
    rollover_risk: float
    stability_index: float
    critical_speed: float
    ackermann_error: float
    max_safe_speed: float
    traction_force_required: float
    vibration_level: float

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ComparisonResult:
    comparison_type: str
    title: str
    subtitle: str
    input_conditions: Dict[str, Any]
    entries: List[Dict[str, Any]]
    winners: Dict[str, str]
    insights: List[str]
    timestamp: float = field(default_factory=timestamp)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class VirtualDriveRequest:
    session_id: str
    vehicle_type: str
    road_type: str
    pole_angle: float
    speed: float
    cargo_mass: float = 0.0
    cargo_offset_lateral: float = 0.0
    cargo_offset_longitudinal: float = 0.0
    cargo_offset_height: float = 0.0
    timestamp: float = field(default_factory=timestamp)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> 'VirtualDriveRequest':
        return cls(
            session_id=data['session_id'],
            vehicle_type=data.get('vehicle_type', 'chariot_double'),
            road_type=data.get('road_type', 'ancient_post_road'),
            pole_angle=float(data['pole_angle']),
            speed=float(data['speed']),
            cargo_mass=float(data.get('cargo_mass', 0)),
            cargo_offset_lateral=float(data.get('cargo_offset_lateral', 0)),
            cargo_offset_longitudinal=float(data.get('cargo_offset_longitudinal', 0)),
            cargo_offset_height=float(data.get('cargo_offset_height', 0)),
            timestamp=float(data.get('timestamp', timestamp()))
        )


@dataclass
class VirtualDriveState:
    session_id: str
    vehicle_type: str
    road_type: str
    x: float
    y: float
    heading: float
    speed: float
    pole_angle: float
    inner_wheel_angle: float
    outer_wheel_angle: float
    turning_radius: float
    roll_angle: float
    roll_rate: float
    yaw_rate: float
    lateral_acceleration: float
    rollover_risk: float
    stability_index: float
    effective_friction: float
    slip_ratio: float
    wheel_rotation: List[float]
    cargo_shift_lateral: float
    cargo_shift_vertical: float
    alert_message: str = ""
    is_tipping: bool = False
    is_stuck: bool = False
    timestamp: float = field(default_factory=timestamp)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def create_response(request_id: str, success: bool, data: Dict[str, Any] = None,
                    error: str = None) -> Dict[str, Any]:
    return {
        'request_id': request_id,
        'success': success,
        'data': data or {},
        'error': error
    }
