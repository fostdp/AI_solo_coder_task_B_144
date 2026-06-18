import math
import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Tuple
import random
import time

from .steering_model import (
    ChariotParams, FourBarLinkageSolver, AckermannSteeringModel,
    MultiBodyDynamicsSteering
)
from .stability_analysis import (
    VehicleDynamicsParams, CargoConfig, StabilityAnalyzer
)
from .message_protocol import (
    VehicleType, RoadSurface, SteeringMechanism,
    RoadEffect, VehicleComparisonEntry, RoadComparisonEntry,
    ComparisonResult, VirtualDriveState, timestamp
)
from .config_loader import get_config_loader


@dataclass
class VehicleFullConfig:
    vehicle_type: str
    name: str
    era: str
    category: str
    description: str
    geometry: ChariotParams
    dynamics: VehicleDynamicsParams
    steering_type: str
    max_steering_angle_deg: float
    max_speed_mps: float
    propulsion: str


class RoadSurfaceModel:
    def __init__(self):
        loader = get_config_loader()
        try:
            sys_cfg = loader.system_config()
            road_file = sys_cfg.get('config_paths', {}).get('road_surfaces', 'road_surface_types')
            if road_file.endswith('.json'):
                road_file = road_file[:-5]
            self._config = loader.load(road_file)
        except Exception:
            self._config = loader.load('road_surface_types')

    def get_surface_config(self, road_type: str) -> Dict[str, Any]:
        return self._config.get(road_type, self._config.get('dirt_road', {}))

    def compute_effects(self, road_type: str, vehicle_dynamics: VehicleDynamicsParams,
                        friction_coeff_override: float = None) -> RoadEffect:
        cfg = self.get_surface_config(road_type)
        if friction_coeff_override is not None:
            mu = friction_coeff_override
        else:
            mu = (cfg['friction_min'] + cfg['friction_max']) / 2
        slip_factor = cfg['slip_factor']
        rolling_resistance = cfg['rolling_resistance']
        bump_amp = cfg['bump_amplitude_m']
        irregularity = cfg['irregularity']

        c_friction = max(0.2, min(1.0, mu / 0.85))
        c_f = vehicle_dynamics.cornering_stiffness_front * c_friction
        c_r = vehicle_dynamics.cornering_stiffness_rear * c_friction

        vibration = irregularity * 2.0 + bump_amp * 10.0

        return RoadEffect(
            friction_coeff=mu,
            rolling_resistance=rolling_resistance,
            slip_factor=slip_factor,
            bump_amplitude=bump_amp,
            irregularity=irregularity,
            road_type=road_type,
            effective_cornering_stiffness_front=c_f,
            effective_cornering_stiffness_rear=c_r,
            vibration_acceleration=vibration
        )

    def list_road_types(self) -> List[Dict[str, Any]]:
        result = []
        for key, v in self._config.items():
            if key.startswith('_') or not isinstance(v, dict) or 'friction_min' not in v:
                continue
            result.append({
                'id': key,
                'name': v.get('name', key),
                'category': v.get('category', ''),
                'description': v.get('description', ''),
                'friction_min': v.get('friction_min', 0),
                'friction_max': v.get('friction_max', 0),
                'slip_factor': v.get('slip_factor', 1.0),
                'rolling_resistance': v.get('rolling_resistance', 0),
                'irregularity': v.get('irregularity', 0)
            })
        return result


class MultiVehicleSteeringModel:
    def __init__(self):
        self._loader = get_config_loader()
        try:
            sys_cfg = self._loader.system_config()
            v_file = sys_cfg.get('config_paths', {}).get('vehicle_types', 'vehicle_types')
            if v_file.endswith('.json'):
                v_file = v_file[:-5]
            self._vehicle_configs = self._loader.load(v_file)
        except Exception:
            self._vehicle_configs = self._loader.load('vehicle_types')
        self._road_model = RoadSurfaceModel()
        self._cache: Dict[str, Tuple[Any, Any, VehicleFullConfig]] = {}

    def list_vehicle_types(self) -> List[Dict[str, Any]]:
        result = []
        for key, v in self._vehicle_configs.items():
            if key.startswith('_') or not isinstance(v, dict) or 'geometry' not in v:
                continue
            geo = v['geometry']
            dyn = v['dynamics']
            result.append({
                'id': key,
                'name': v['name'],
                'era': v.get('era', '未知'),
                'category': v.get('category', ''),
                'description': v.get('description', ''),
                'steering_type': v.get('steering_type', ''),
                'max_steering_angle_deg': v.get('max_steering_angle_deg', 30),
                'max_speed_mps': v.get('max_speed_mps', 0),
                'propulsion': v.get('propulsion', ''),
                'wheelbase': geo.get('wheelbase', 0),
                'track_width': geo.get('track_width', 0),
                'wheel_radius': geo.get('wheel_radius', 0),
                'mass': dyn.get('mass', 0),
                'cg_height': dyn.get('cg_height', 0)
            })
        return result

    def get_vehicle_config(self, vehicle_type: str) -> Optional[VehicleFullConfig]:
        if vehicle_type not in self._vehicle_configs:
            return None
        v = self._vehicle_configs[vehicle_type]
        geo = v['geometry']
        dyn = v['dynamics']
        return VehicleFullConfig(
            vehicle_type=vehicle_type,
            name=v['name'],
            era=v['era'],
            category=v['category'],
            description=v['description'],
            geometry=ChariotParams(
                wheelbase=geo['wheelbase'],
                track_width=geo['track_width'],
                wheel_radius=geo['wheel_radius'],
                pole_length=geo.get('pole_length', 1.8),
                kingpin_offset=geo.get('kingpin_offset', 0.1),
                steering_arm_length=geo.get('steering_arm_length', 0.25),
                tie_rod_length=geo.get('tie_rod_length', -1.0),
                ackermann_angle_deg=geo.get('ackermann_angle_deg', 12.0)
            ),
            dynamics=VehicleDynamicsParams(
                wheelbase=geo['wheelbase'],
                track_width=geo['track_width'],
                cg_height=dyn['cg_height'],
                cg_longitudinal=dyn.get('cg_longitudinal', 0.0),
                cg_lateral=dyn.get('cg_lateral', 0.0),
                roll_center_height=dyn.get('roll_center_height', 0.3),
                mass=dyn['mass'],
                yaw_inertia=dyn.get('yaw_inertia', 1200.0),
                roll_stiffness=dyn.get('roll_stiffness', 30000.0),
                damping_ratio=dyn.get('damping_ratio', 0.3),
                wheel_radius=geo['wheel_radius'],
                cornering_stiffness_front=dyn.get('cornering_stiffness_front', 25000.0),
                cornering_stiffness_rear=dyn.get('cornering_stiffness_rear', 35000.0)
            ),
            steering_type=v['steering_type'],
            max_steering_angle_deg=v['max_steering_angle_deg'],
            max_speed_mps=v['max_speed_mps'],
            propulsion=v['propulsion']
        )

    def _build_models(self, vehicle_type: str):
        if vehicle_type in self._cache:
            return self._cache[vehicle_type]
        config = self.get_vehicle_config(vehicle_type)
        if not config:
            return None, None, None
        if config.steering_type == 'single_wheel_direct':
            ackermann = SingleWheelDirectSteering(config.geometry)
            dynamics = None
        elif config.steering_type == 'front_axle_ackermann':
            four_bar = FourBarLinkageSolver(config.geometry)
            ackermann = FrontAxleAckermannModel(config.geometry, four_bar)
            dynamics = DynamicsSteeringAdapter(config.geometry)
        elif config.steering_type == 'rack_pinion_ackermann':
            ackermann = RackPinionSteeringModel(config.geometry)
            dynamics = DynamicsSteeringAdapter(config.geometry)
        else:
            ackermann = AckermannSteeringAdapter(config.geometry)
            dynamics = DynamicsSteeringAdapter(config.geometry)

        self._cache[vehicle_type] = (ackermann, dynamics, config)
        return ackermann, dynamics, config

    def compute_steering(self, vehicle_type: str, pole_angle_deg: float,
                         speed_mps: float, friction_coeff: float,
                         road_type: str = 'dirt_road'):
        ackermann, dynamics, config = self._build_models(vehicle_type)
        if not ackermann:
            return None

        speed_capped = min(speed_mps, config.max_speed_mps)
        max_angle = config.max_steering_angle_deg
        pole_clamped = max(-max_angle, min(max_angle, pole_angle_deg))

        steering = ackermann.compute_inner_outer_from_pole(
            math.radians(pole_clamped), speed_capped
        )

        road_effect = self._road_model.compute_effects(road_type, config.dynamics, friction_coeff)
        mu = road_effect.friction_coeff

        if dynamics:
            final_steering = dynamics.compute_corrected_steering(
                pole_angle_rad=math.radians(pole_clamped),
                vehicle_speed=speed_capped,
                friction_coeff=mu,
                cargo_mass=0.0
            )
            trajectories = dynamics.compute_wheel_trajectories(
                steering_result=final_steering,
                speed=speed_capped,
                duration_sec=3.0,
                dt=0.05
            )
        else:
            final_steering = steering
            trajectories = None

        result = {
            'vehicle_type': vehicle_type,
            'vehicle_name': config.name,
            'steering_type': config.steering_type,
            'inner_wheel_angle': math.degrees(final_steering.inner_wheel_angle),
            'outer_wheel_angle': math.degrees(final_steering.outer_wheel_angle),
            'turning_radius': final_steering.turning_radius,
            'wheel_speed_diff': final_steering.wheel_speed_diff,
            'ackermann_error': final_steering.ackermann_error,
            'pole_effective_angle': math.degrees(final_steering.pole_effective_angle),
            'transmission_angle_inner': math.degrees(getattr(final_steering, 'transmission_angle_inner', 0)),
            'transmission_angle_outer': math.degrees(getattr(final_steering, 'transmission_angle_outer', 0)),
            'linkage_interference': getattr(final_steering, 'linkage_interference', False),
            'dead_point_risk': getattr(final_steering, 'dead_point_risk', False),
            'degradation_level': getattr(final_steering, 'degradation_level', 0),
            'max_inner_wheel_angle_deg': ackermann.get_max_safe_angle() if hasattr(ackermann, 'get_max_safe_angle') else max_angle,
            'friction_coeff_used': mu,
            'road_type': road_type,
            'wheel_trajectories': trajectories
        }
        return result

    def compute_stability(self, vehicle_type: str, pole_angle_deg: float,
                          speed_mps: float, roll_angle_deg: float,
                          friction_coeff: float, cargo: CargoConfig = None,
                          road_type: str = 'dirt_road'):
        _, _, config = self._build_models(vehicle_type)
        if not config:
            return None
        speed_capped = min(speed_mps, config.max_speed_mps)
        road_effect = self._road_model.compute_effects(road_type, config.dynamics, friction_coeff)

        modified_dynamics = VehicleDynamicsParams(
            wheelbase=max(0.01, config.dynamics.wheelbase),
            track_width=max(0.01, config.dynamics.track_width),
            cg_height=config.dynamics.cg_height,
            cg_longitudinal=config.dynamics.cg_longitudinal,
            cg_lateral=config.dynamics.cg_lateral,
            roll_center_height=config.dynamics.roll_center_height,
            mass=config.dynamics.mass,
            yaw_inertia=config.dynamics.yaw_inertia,
            roll_stiffness=config.dynamics.roll_stiffness,
            damping_ratio=config.dynamics.damping_ratio,
            wheel_radius=config.dynamics.wheel_radius,
            cornering_stiffness_front=max(500.0, road_effect.effective_cornering_stiffness_front),
            cornering_stiffness_rear=max(500.0, road_effect.effective_cornering_stiffness_rear)
        )

        analyzer = StabilityAnalyzer(modified_dynamics, cargo or CargoConfig())
        max_angle = config.max_steering_angle_deg
        pole_clamped = max(-max_angle, min(max_angle, pole_angle_deg))

        steering_result = self.compute_steering(
            vehicle_type, pole_angle_deg, speed_mps, friction_coeff, road_type
        )
        if not steering_result:
            return None

        stab = analyzer.analyze(
            speed=speed_capped,
            pole_angle_deg=pole_clamped,
            roll_angle_deg=roll_angle_deg,
            slip_rate=0.1,
            friction_coeff=road_effect.friction_coeff,
            dt=0.05,
            vertical_accel=road_effect.vibration_acceleration
        )

        ssf = modified_dynamics.track_width / (2 * stab.effective_cg_height)

        return {
            'vehicle_type': vehicle_type,
            'vehicle_name': config.name,
            'roll_angle': stab.roll_angle,
            'roll_rate': stab.roll_rate,
            'yaw_rate': stab.yaw_rate,
            'lateral_acceleration': stab.lateral_acceleration,
            'roll_center_height': stab.roll_center_height,
            'rollover_risk': stab.rollover_risk,
            'stability_index': stab.stability_index,
            'understeer_gradient': stab.understeer_gradient,
            'critical_speed': stab.critical_speed,
            'effective_cg_height': stab.effective_cg_height,
            'effective_cg_lateral': stab.effective_cg_lateral,
            'effective_cg_longitudinal': stab.effective_cg_longitudinal,
            'effective_yaw_inertia': stab.effective_yaw_inertia,
            'cargo_shift_lateral': stab.cargo_shift_lateral,
            'cargo_shift_vertical': stab.cargo_shift_vertical,
            'ssf_static': ssf,
            'max_speed_mps': config.max_speed_mps,
            'mass': config.dynamics.mass,
            'cg_height': config.dynamics.cg_height,
            'wheelbase': config.dynamics.wheelbase,
            'track_width': config.dynamics.track_width,
            'propulsion': config.propulsion,
            'friction_coeff_used': road_effect.friction_coeff,
            'road_type': road_type,
            'vibration_level': road_effect.vibration_acceleration
        }


class AckermannSteeringAdapter:
    def __init__(self, params: ChariotParams):
        self._inner = AckermannSteeringModel(params)
        self.params = params

    def compute_inner_outer_from_pole(self, pole_angle_rad: float, speed: float):
        result = self._inner.calculate_ackermann_geometry(pole_angle_rad)
        return result

    def get_max_safe_angle(self) -> float:
        return 35.0


class DynamicsSteeringAdapter:
    def __init__(self, params: ChariotParams):
        self._inner = MultiBodyDynamicsSteering(params)
        self.params = params

    def compute_corrected_steering(self, pole_angle_rad: float, vehicle_speed: float,
                                   friction_coeff: float, cargo_mass: float = 0.0):
        return self._inner.calculate_steering(pole_angle_rad, vehicle_speed, friction_coeff)

    def compute_wheel_trajectories(self, steering_result, speed: float,
                                   duration_sec: float = 3.0, dt: float = 0.05):
        try:
            traj = self._inner.get_wheel_trajectory(
                steering_result.pole_effective_angle if hasattr(steering_result, 'pole_effective_angle') else 0,
                speed, duration_sec, dt
            )
            return traj
        except Exception:
            return None


class SingleWheelDirectSteering:
    def __init__(self, params: ChariotParams):
        self.params = params

    def compute_inner_outer_from_pole(self, pole_angle_rad: float, speed: float):
        wheel_angle = pole_angle_rad
        if abs(pole_angle_rad) < 1e-6:
            R = float('inf')
        else:
            R = 0.8 / math.tan(pole_angle_rad)
        return type('SteeringResult', (), {
            'inner_wheel_angle': wheel_angle,
            'outer_wheel_angle': wheel_angle,
            'turning_radius': R,
            'wheel_speed_diff': 0.0,
            'ackermann_error': 0.0,
            'pole_effective_angle': pole_angle_rad,
            'transmission_angle_inner': math.radians(90),
            'transmission_angle_outer': math.radians(90),
            'linkage_interference': False,
            'dead_point_risk': False,
            'degradation_level': 0
        })()

    def get_max_safe_angle(self):
        return 75.0


class FrontAxleAckermannModel:
    def __init__(self, params: ChariotParams, four_bar: FourBarLinkageSolver):
        self.params = params
        self.four_bar = four_bar

    def compute_inner_outer_from_pole(self, pole_angle_rad: float, speed: float):
        T = self.params.track_width
        L = self.params.wheelbase
        if abs(pole_angle_rad) < 1e-3:
            delta_i = 0.0
            delta_o = 0.0
            R = float('inf')
        else:
            R_ideal = L / math.tan(pole_angle_rad)
            R_inner = max(1.0, abs(R_ideal) - T / 2)
            sign = 1 if pole_angle_rad > 0 else -1
            delta_i = sign * math.atan(L / R_inner)
            max_delta = math.radians(30)
            delta_i = max(-max_delta, min(max_delta, delta_i))
            if abs(delta_i) > 0.01:
                cot_o = 1.0 / math.tan(delta_i) + T / L
                delta_o = sign * math.atan(1.0 / cot_o)
            else:
                delta_o = delta_i
            R = L / math.tan((delta_i + delta_o) / 2) if abs((delta_i + delta_o) / 2) > 0.001 else float('inf')

        ack_err = 0.0
        if abs(delta_i) > 0.01 and abs(delta_o) > 0.01:
            ideal_cot_diff = T / L
            actual_cot_diff = (1.0 / math.tan(delta_o)) - (1.0 / math.tan(delta_i))
            ack_err = abs(actual_cot_diff - ideal_cot_diff) / ideal_cot_diff

        return type('SteeringResult', (), {
            'inner_wheel_angle': delta_i,
            'outer_wheel_angle': delta_o,
            'turning_radius': R,
            'wheel_speed_diff': 0.0,
            'ackermann_error': ack_err,
            'pole_effective_angle': pole_angle_rad,
            'transmission_angle_inner': math.radians(75),
            'transmission_angle_outer': math.radians(75),
            'linkage_interference': False,
            'dead_point_risk': abs(delta_i) > math.radians(28),
            'degradation_level': 0
        })()

    def get_max_safe_angle(self):
        return 28.0


class RackPinionSteeringModel:
    def __init__(self, params: ChariotParams):
        self.params = params

    def compute_inner_outer_from_pole(self, pole_angle_rad: float, speed: float):
        T = self.params.track_width
        L = self.params.wheelbase
        steering_ratio = 1.0
        road_wheel_angle = pole_angle_rad / steering_ratio
        max_delta = math.radians(40)
        delta_i = max(-max_delta, min(max_delta, road_wheel_angle))

        if abs(delta_i) < 0.01:
            delta_o = delta_i
            R = float('inf')
        else:
            sign = 1 if delta_i > 0 else -1
            cot_o = 1.0 / math.tan(abs(delta_i)) + T / L
            delta_o = sign * math.atan(1.0 / cot_o)
            correction = 0.92
            delta_o = delta_o * correction + delta_i * (1 - correction)
            R = L / math.tan((delta_i + delta_o) / 2)

        ack_err = 0.0
        if abs(delta_i) > 0.01 and abs(delta_o) > 0.01:
            ideal_cot_diff = T / L
            actual_cot_diff = (1.0 / math.tan(delta_o)) - (1.0 / math.tan(delta_i))
            ack_err = abs(actual_cot_diff - ideal_cot_diff) / ideal_cot_diff

        return type('SteeringResult', (), {
            'inner_wheel_angle': delta_i,
            'outer_wheel_angle': delta_o,
            'turning_radius': R,
            'wheel_speed_diff': 0.0,
            'ackermann_error': ack_err,
            'pole_effective_angle': pole_angle_rad,
            'transmission_angle_inner': math.radians(85),
            'transmission_angle_outer': math.radians(85),
            'linkage_interference': False,
            'dead_point_risk': False,
            'degradation_level': 0
        })()

    def get_max_safe_angle(self):
        return 40.0


class ComparisonAnalyzer:
    """
    [重构后 v2.0] 对比分析器
    - compare_vehicles 路由到 steering_comparator.SteeringComparator
    - compare_road_surfaces 路由到 road_simulator.RoadSimulator
    保持原有 API 签名 100% 不变，确保后向兼容。
    """

    def __init__(self):
        # 延迟导入，避免循环依赖
        from .steering_comparator import SteeringComparator
        from .era_comparator import EraComparator
        from .road_simulator import RoadSimulator
        self._steering_cmp = SteeringComparator()
        self._era_cmp = EraComparator()
        self._road_sim = RoadSimulator()
        self._steering = self._steering_cmp._steering
        self._road = self._road_sim._road

    def compare_vehicles(self, vehicle_types, pole_angle_deg, speed_mps,
                         friction_coeff, road_type='dirt_road', cargo=None):
        return self._steering_cmp.compare(
            vehicle_types, pole_angle_deg, speed_mps, friction_coeff,
            road_type, cargo
        )

    def compare_eras(self, ancient_types=None, modern_types=None,
                     pole_angle_deg=20.0, speed_mps=5.0,
                     friction_coeff=0.7, road_type='ancient_post_road',
                     cargo=None):
        return self._era_cmp.compare_eras(
            ancient_types, modern_types, pole_angle_deg, speed_mps,
            friction_coeff, road_type, cargo
        )

    def compare_road_surfaces(self, vehicle_type, pole_angle_deg,
                              speed_mps, road_types, cargo=None):
        return self._road_sim.compare_roads(
            vehicle_type, pole_angle_deg, speed_mps, road_types, cargo
        )


class ForceFeedbackModel:
    """
    力反馈方向盘模型 v2.0
    基于SAE J2181和ISO 11663标准
    4项分量：回正力矩 + 阻尼力矩 + 路感振动 + 库仑摩擦
    """
    def __init__(self):
        self._prev_pole_angle_rad: float = 0.0
        self._prev_time: float = time.time()
        self._road_feel_filter: float = 0.0
        self._caster_angle_rad = math.radians(3.5)
        self._mechanical_trail_m = 0.025
        self._pneumatic_trail_coeff = 0.45
        self._steering_ratio = 16.5

    def compute(self, vehicle_dynamics: VehicleDynamicsParams,
                road_cfg: Dict[str, Any],
                pole_angle_deg: float,
                wheel_angle_avg_rad: float,
                speed_mps: float,
                lateral_accel_mps2: float,
                slip_ratio: float,
                mu: float,
                cornering_stiff_front_N_per_rad: float,
                dt: float = 0.05) -> Dict[str, float]:

        pole_rad = math.radians(pole_angle_deg)
        now = time.time()
        d_angle = pole_rad - self._prev_pole_angle_rad
        dt_actual = max(0.001, now - self._prev_time)
        steer_angvel_radps = d_angle / dt_actual
        self._prev_pole_angle_rad = pole_rad
        self._prev_time = now

        if abs(wheel_angle_avg_rad) > 0.001 and speed_mps > 0.1:
            slip_angle_front_rad = wheel_angle_avg_rad - math.atan(
                (vehicle_dynamics.wheelbase * 0.5 * (lateral_accel_mps2 / (9.81 * vehicle_dynamics.cg_height + 1e-6)) + 0)
                / max(0.1, speed_mps)
            )
        else:
            slip_angle_front_rad = wheel_angle_avg_rad
        slip_angle_front_rad = max(-0.3, min(0.3, slip_angle_front_rad))

        lateral_force_front_N = cornering_stiff_front_N_per_rad * slip_angle_front_rad * (1.0 - 0.5 * slip_ratio)
        pneumatic_trail_m = self._pneumatic_trail_coeff * vehicle_dynamics.wheel_radius * max(0.0, 1.0 - abs(slip_angle_front_rad) / 0.3)
        total_trail_m = (self._mechanical_trail_m
                         + vehicle_dynamics.cg_height * math.tan(self._caster_angle_rad)
                         + pneumatic_trail_m)
        aligning_torque_Nm = -lateral_force_front_N * total_trail_m * max(0.1, mu / 0.85)

        steering_viscosity_Nm_per_radps = 0.8
        damping_torque_Nm = -steering_viscosity_Nm_per_radps * steer_angvel_radps * max(0.1, speed_mps / 10.0)

        mu_clamped = max(0.09, min(0.95, mu))
        friction_static_Nm = 1.2 + 2.8 * (1.0 - (mu_clamped - 0.09) / 0.86)
        if abs(steer_angvel_radps) > 0.001:
            friction_torque_Nm = -friction_static_Nm * math.tanh(steer_angvel_radps / 0.02)
        else:
            friction_torque_Nm = -friction_static_Nm * 0.5 * math.tanh(pole_rad / 0.005) if abs(pole_rad) < 0.05 else 0.0

        bump = road_cfg.get('bump_amplitude_m', 0.02)
        irregular = road_cfg.get('irregularity', 0.0)
        f_low = 10.0
        f_high = 55.0
        omega = 2.0 * math.pi * random.uniform(f_low, f_high)
        amplitude_gain = (bump * 5000.0 + irregular * 3.5) * min(1.0, speed_mps / 5.0)
        white_noise = random.uniform(-1.0, 1.0)
        road_feel_raw_Nm = amplitude_gain * white_noise * math.sin(omega * now)
        alpha = 0.3
        self._road_feel_filter = alpha * road_feel_raw_Nm + (1.0 - alpha) * self._road_feel_filter
        road_feel_Nm = self._road_feel_filter

        eps = 0.02
        if speed_mps < 0.5 and abs(pole_rad) < 0.02:
            total = 0.0
            aligning_torque_Nm = 0.0
            damping_torque_Nm = 0.0
            friction_torque_Nm = 0.0
            road_feel_Nm = 0.0
        else:
            total = aligning_torque_Nm + damping_torque_Nm + road_feel_Nm + friction_torque_Nm

        max_allowable_Nm = 8.0
        if abs(total) > max_allowable_Nm:
            scale = max_allowable_Nm / abs(total)
            aligning_torque_Nm *= scale
            damping_torque_Nm *= scale
            road_feel_Nm *= scale
            friction_torque_Nm *= scale
            total *= scale

        intensity = max(0.0, min(1.0, abs(total) / max_allowable_Nm))

        return {
            'ffb_total_torque': total,
            'ffb_aligning_torque': aligning_torque_Nm,
            'ffb_damping_torque': damping_torque_Nm,
            'ffb_road_feel_torque': road_feel_Nm,
            'ffb_friction_torque': friction_torque_Nm,
            'ffb_intensity': intensity,
            '_debug_slip_angle_deg': math.degrees(slip_angle_front_rad),
            '_debug_lateral_force_N': lateral_force_front_N,
            '_debug_total_trail_m': total_trail_m
        }


class VirtualDriveEngine:
    """
    [重构后 v2.0] 虚拟驾驶引擎薄封装
    完整逻辑已迁移到 vr_chariot.VRChariotEngine
    - 保持原有 step / reset_session API 100% 不变，确保后向兼容
    """

    def __init__(self):
        from .vr_chariot import VRChariotEngine
        self._inner = VRChariotEngine()
        self._steering = self._inner._steering
        self._road = self._inner._road
        self._ffb = self._inner._ffb
        self._sessions = self._inner._sessions

    def step(self, session_id, vehicle_type, road_type,
             pole_angle_deg, throttle, brake,
             cargo=None, dt=0.05):
        return self._inner.step(
            session_id, vehicle_type, road_type,
            pole_angle_deg, throttle, brake, cargo, dt
        )

    def reset_session(self, session_id):
        self._inner.reset_session(session_id)
