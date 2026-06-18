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
        return [
            {
                'id': key,
                'name': v['name'],
                'category': v['category'],
                'description': v['description'],
                'friction_min': v['friction_min'],
                'friction_max': v['friction_max'],
                'slip_factor': v['slip_factor'],
                'rolling_resistance': v['rolling_resistance'],
                'irregularity': v['irregularity']
            }
            for key, v in self._config.items()
        ]


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
            geo = v['geometry']
            dyn = v['dynamics']
            result.append({
                'id': key,
                'name': v['name'],
                'era': v['era'],
                'category': v['category'],
                'description': v['description'],
                'steering_type': v['steering_type'],
                'max_steering_angle_deg': v['max_steering_angle_deg'],
                'max_speed_mps': v['max_speed_mps'],
                'propulsion': v['propulsion'],
                'wheelbase': geo['wheelbase'],
                'track_width': geo['track_width'],
                'wheel_radius': geo['wheel_radius'],
                'mass': dyn['mass'],
                'cg_height': dyn['cg_height']
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
    def __init__(self):
        self._steering = MultiVehicleSteeringModel()
        self._road = RoadSurfaceModel()

    def compare_vehicles(self, vehicle_types: List[str], pole_angle_deg: float,
                         speed_mps: float, friction_coeff: float,
                         road_type: str = 'dirt_road',
                         cargo: CargoConfig = None) -> ComparisonResult:
        entries = []
        for vt in vehicle_types:
            steering = self._steering.compute_steering(
                vt, pole_angle_deg, speed_mps, friction_coeff, road_type
            )
            stability = self._steering.compute_stability(
                vt, pole_angle_deg, speed_mps, 0.0, friction_coeff, cargo, road_type
            )
            if not steering or not stability:
                continue
            entries.append(VehicleComparisonEntry(
                vehicle_type=vt,
                vehicle_name=steering['vehicle_name'],
                era=self._steering.get_vehicle_config(vt).era,
                category=self._steering.get_vehicle_config(vt).category,
                steering_mechanism=steering['steering_type'],
                inner_wheel_angle=steering['inner_wheel_angle'],
                outer_wheel_angle=steering['outer_wheel_angle'],
                turning_radius=steering['turning_radius'],
                ackermann_error=steering['ackermann_error'],
                max_inner_wheel_angle=steering['max_inner_wheel_angle_deg'],
                min_turning_radius=self._estimate_min_turning_radius(vt),
                transmission_angle_min=min(
                    steering.get('transmission_angle_inner', 75.0),
                    steering.get('transmission_angle_outer', 75.0)
                ),
                yaw_rate=stability['yaw_rate'],
                lateral_acceleration=stability['lateral_acceleration'],
                rollover_risk=stability['rollover_risk'],
                stability_index=stability['stability_index'],
                critical_speed=stability['critical_speed'],
                ssf_static=stability['ssf_static'],
                understeer_gradient=stability['understeer_gradient'],
                max_speed_mps=stability['max_speed_mps'],
                mass=stability['mass'],
                cg_height=stability['cg_height'],
                wheelbase=stability['wheelbase'],
                track_width=stability['track_width'],
                propulsion=stability['propulsion']
            ).to_dict())

        winners = self._find_vehicle_winners(entries)
        insights = self._generate_vehicle_insights(entries, pole_angle_deg, speed_mps, road_type)

        return ComparisonResult(
            comparison_type='vehicle',
            title='古代车辆与现代汽车转向机构对比分析',
            subtitle=f'辕杆角 {pole_angle_deg}° | 车速 {speed_mps} m/s | 路面: {self._road.get_surface_config(road_type).get("name", road_type)}',
            input_conditions={
                'pole_angle_deg': pole_angle_deg,
                'speed_mps': speed_mps,
                'friction_coeff': friction_coeff,
                'road_type': road_type
            },
            entries=entries,
            winners=winners,
            insights=insights
        )

    def _estimate_min_turning_radius(self, vehicle_type: str) -> float:
        config = self._steering.get_vehicle_config(vehicle_type)
        if not config:
            return 999.0
        max_delta = math.radians(config.max_steering_angle_deg)
        if abs(max_delta) < 0.01:
            return float('inf')
        return config.geometry.wheelbase / math.tan(max_delta * 0.85)

    def _find_vehicle_winners(self, entries: List[Dict]) -> Dict[str, str]:
        winners = {}
        if not entries:
            return winners
        categories = [
            ('最小转弯半径', 'min_turning_radius', True),
            ('最高临界速度', 'critical_speed', False),
            ('最低阿克曼误差', 'ackermann_error', True),
            ('最高稳定性指数', 'stability_index', False),
            ('最低侧翻风险', 'rollover_risk', True),
            ('最高静态稳定系数SSF', 'ssf_static', False),
            ('最大内轮转角', 'max_inner_wheel_angle', False),
            ('最高最高车速', 'max_speed_mps', False)
        ]
        for name, key, lower_is_better in categories:
            sorted_entries = sorted(entries, key=lambda e: e.get(key, float('inf') if lower_is_better else 0), reverse=not lower_is_better)
            winners[name] = sorted_entries[0]['vehicle_name']
        return winners

    def _generate_vehicle_insights(self, entries: List[Dict], pole_deg: float,
                                    speed: float, road_type: str) -> List[str]:
        insights = []
        if len(entries) < 2:
            return insights
        ancient = [e for e in entries if e.get('era') == '古代']
        modern = [e for e in entries if e.get('era') == '现代']

        if ancient and modern:
            avg_radius_ancient = sum(e['turning_radius'] for e in ancient if e['turning_radius'] != float('inf')) / max(1, len(ancient))
            avg_radius_modern = sum(e['turning_radius'] for e in modern if e['turning_radius'] != float('inf')) / max(1, len(modern))
            if avg_radius_ancient < avg_radius_modern:
                insights.append(f"在当前工况下，古代车辆平均转弯半径({avg_radius_ancient:.1f}m)比现代汽车({avg_radius_modern:.1f}m)更紧凑，显示出古代城市狭窄街道适应性设计。")
            else:
                insights.append(f"现代汽车平均转弯半径({avg_radius_modern:.1f}m)优于古代车辆({avg_radius_ancient:.1f}m)，齿轮齿条转向精度更高。")

            avg_stab_ancient = sum(e['stability_index'] for e in ancient) / len(ancient)
            avg_stab_modern = sum(e['stability_index'] for e in modern) / len(modern)
            insights.append(f"现代汽车稳定性指数({avg_stab_modern:.2f})比古代车辆({avg_stab_ancient:.2f})高出{((avg_stab_modern - avg_stab_ancient) / max(0.01, avg_stab_ancient) * 100):.0f}%，低重心和独立悬架设计贡献显著。")

            max_speed_ancient = max(e['max_speed_mps'] for e in ancient) if ancient else 0
            max_speed_modern = max(e['max_speed_mps'] for e in modern) if modern else 0
            insights.append(f"动力系统差异：古代畜力最高约{max_speed_ancient * 3.6:.0f}km/h，现代内燃机可达{max_speed_modern * 3.6:.0f}km/h，速度提升约{(max_speed_modern / max(0.1, max_speed_ancient)):.1f}倍。")

        for e in entries:
            if e.get('ackermann_error', 0) > 0.05:
                insights.append(f"{e['vehicle_name']}阿克曼误差为{e['ackermann_error']*100:.1f}%，高速转弯时轮胎磨损较明显。")
            if e.get('rollover_risk', 0) > 50:
                insights.append(f"⚠ {e['vehicle_name']}在当前工况下侧翻风险达{e['rollover_risk']:.0f}%，重心偏高是主要原因。")

        wheelbarrow = [e for e in entries if e.get('category') == '独轮车']
        if wheelbarrow:
            w = wheelbarrow[0]
            insights.append(f"独轮车转弯半径仅{w['turning_radius'] if w['turning_radius'] != float('inf') else '∞'}m，原地转向能力极强，但侧翻风险{w['rollover_risk']:.0f}%，需极高驾驶技巧。")

        if not insights:
            insights.append("各车辆在当前工况下表现均衡，可调整参数观察差异。")
        return insights

    def compare_road_surfaces(self, vehicle_type: str, pole_angle_deg: float,
                              speed_mps: float, road_types: List[str],
                              cargo: CargoConfig = None) -> ComparisonResult:
        entries = []
        for rt in road_types:
            cfg = self._road.get_surface_config(rt)
            mu = (cfg['friction_min'] + cfg['friction_max']) / 2
            steering = self._steering.compute_steering(
                vehicle_type, pole_angle_deg, speed_mps, mu, rt
            )
            stability = self._steering.compute_stability(
                vehicle_type, pole_angle_deg, speed_mps, 0.0, mu, cargo, rt
            )
            if not steering or not stability:
                continue
            vehicle_config = self._steering.get_vehicle_config(vehicle_type)
            effective_speed = speed_mps / cfg.get('slip_factor', 1.0)
            traction_force = cfg.get('rolling_resistance', 0.05) * vehicle_config.dynamics.mass * 9.81
            entries.append(RoadComparisonEntry(
                road_type=rt,
                road_name=cfg.get('name', rt),
                category=cfg.get('category', ''),
                friction_coeff=mu,
                rolling_resistance=cfg.get('rolling_resistance', 0),
                slip_factor=cfg.get('slip_factor', 1.0),
                effective_speed=effective_speed,
                turning_radius_effective=steering['turning_radius'] * cfg.get('slip_factor', 1.0),
                yaw_rate=stability['yaw_rate'],
                lateral_acceleration=stability['lateral_acceleration'],
                rollover_risk=stability['rollover_risk'],
                stability_index=stability['stability_index'],
                critical_speed=stability['critical_speed'],
                ackermann_error=steering['ackermann_error'],
                max_safe_speed=min(stability['critical_speed'], vehicle_config.max_speed_mps),
                traction_force_required=traction_force,
                vibration_level=stability['vibration_level']
            ).to_dict())

        winners = self._find_road_winners(entries)
        insights = self._generate_road_insights(entries, vehicle_type, pole_angle_deg, speed_mps)

        return ComparisonResult(
            comparison_type='road',
            title='不同路面条件操控稳定性对比',
            subtitle=f'车辆: {self._steering.get_vehicle_config(vehicle_type).name if self._steering.get_vehicle_config(vehicle_type) else vehicle_type} | 辕杆角 {pole_angle_deg}° | 车速 {speed_mps} m/s',
            input_conditions={
                'vehicle_type': vehicle_type,
                'pole_angle_deg': pole_angle_deg,
                'speed_mps': speed_mps
            },
            entries=entries,
            winners=winners,
            insights=insights
        )

    def _find_road_winners(self, entries: List[Dict]) -> Dict[str, str]:
        winners = {}
        if not entries:
            return winners
        categories = [
            ('最高摩擦系数', 'friction_coeff', False),
            ('最低滚动阻力', 'rolling_resistance', True),
            ('最低侧翻风险', 'rollover_risk', True),
            ('最高稳定性指数', 'stability_index', False),
            ('最高安全车速', 'max_safe_speed', False),
            ('最小有效转弯半径', 'turning_radius_effective', True),
            ('最低牵引力需求', 'traction_force_required', True),
            ('最低颠簸振动', 'vibration_level', True)
        ]
        for name, key, lower_is_better in categories:
            sorted_entries = sorted(
                entries,
                key=lambda e: e.get(key, float('inf') if lower_is_better else 0),
                reverse=not lower_is_better
            )
            winners[name] = sorted_entries[0]['road_name']
        return winners

    def _generate_road_insights(self, entries: List[Dict], vehicle_type: str,
                                 pole_deg: float, speed: float) -> List[str]:
        insights = []
        if not entries:
            return insights

        best = min(entries, key=lambda e: e.get('rollover_risk', 100))
        worst = max(entries, key=lambda e: e.get('rollover_risk', 0))
        insights.append(
            f"最佳路面【{best['road_name']}】侧翻风险仅{best['rollover_risk']:.0f}%，"
            f"最差路面【{worst['road_name']}】高达{worst['rollover_risk']:.0f}%，"
            f"差距{worst['rollover_risk'] - best['rollover_risk']:.0f}个百分点。"
        )

        if worst.get('rollover_risk', 0) > 70 and speed > 3:
            insights.append(
                f"⚠ 在【{worst['road_name']}】上以{speed}m/s行驶存在严重危险，"
                f"建议将车速降至{max(1, int(worst.get('max_safe_speed', 2)))}m/s以下。"
            )

        pavement = [e for e in entries if '铺装' in e.get('category', '')]
        unpaved = [e for e in entries if '非铺装' in e.get('category', '')]
        if pavement and unpaved:
            avg_stab_p = sum(e['stability_index'] for e in pavement) / len(pavement)
            avg_stab_u = sum(e['stability_index'] for e in unpaved) / len(unpaved)
            insights.append(
                f"铺装路面平均稳定性指数({avg_stab_p:.2f})明显优于非铺装路面({avg_stab_u:.2f})，"
                f"秦汉驿道系统的建设对运输效率提升显著。"
            )

        mud = [e for e in entries if '泥泞' in e.get('road_name', '')]
        if mud:
            m = mud[0]
            insights.append(
                f"泥泞路牵引力需求达{m['traction_force_required']:.0f}N，"
                f"约为干燥路面的{m['traction_force_required'] / max(1, best.get('traction_force_required', 100)):.1f}倍，"
                f"需要牲畜数量成倍增加。"
            )

        stone = [e for e in entries if '石板' in e.get('road_name', '')]
        if stone:
            s = stone[0]
            insights.append(
                f"古代石板路振动等级{s['vibration_level']:.1f}，"
                f"对易碎货物和乘客舒适度影响较大，但摩擦系数优于土路。"
            )

        if not insights or len(insights) < 2:
            insights.append("增加车速或辕杆角度可放大各路面间的性能差异。")
        return insights


class VirtualDriveEngine:
    def __init__(self):
        self._steering = MultiVehicleSteeringModel()
        self._road = RoadSurfaceModel()
        self._sessions: Dict[str, Dict[str, Any]] = {}

    def _get_or_create_session(self, session_id: str) -> Dict[str, Any]:
        if session_id not in self._sessions:
            self._sessions[session_id] = {
                'x': 0.0,
                'y': 0.0,
                'heading': 0.0,
                'speed': 0.0,
                'wheel_rotations': [0.0, 0.0, 0.0, 0.0],
                'last_update': time.time(),
                'cargo_shift_lateral': 0.0,
                'cargo_shift_vertical': 0.0,
                'cargo_velocity_lat': 0.0,
                'cargo_velocity_ver': 0.0
            }
        return self._sessions[session_id]

    def step(self, session_id: str, vehicle_type: str, road_type: str,
             pole_angle_deg: float, throttle: float, brake: float,
             cargo: CargoConfig = None, dt: float = 0.05) -> VirtualDriveState:
        session = self._get_or_create_session(session_id)
        config = self._steering.get_vehicle_config(vehicle_type)
        if not config:
            config = self._steering.get_vehicle_config('chariot_double')
        cfg = self._road.get_surface_config(road_type)
        mu = (cfg['friction_min'] + cfg['friction_max']) / 2
        roll_resist = cfg.get('rolling_resistance', 0.05)
        slip_factor = cfg.get('slip_factor', 1.0)
        bump_amp = cfg.get('bump_amplitude_m', 0.02)

        accel_target = throttle * 2.0
        brake_force = brake * 5.0
        rolling_force = roll_resist * 9.81
        session['speed'] += (accel_target - rolling_force - brake_force) * dt
        session['speed'] = max(0.0, min(session['speed'], config.max_speed_mps))

        speed = session['speed']
        steering = self._steering.compute_steering(
            vehicle_type, pole_angle_deg, speed, mu, road_type
        )
        if steering and steering['turning_radius'] != float('inf') and abs(steering['turning_radius']) > 0.5:
            R_eff = steering['turning_radius'] * slip_factor
            yaw_rate = speed / R_eff
        else:
            R_eff = float('inf')
            yaw_rate = 0.0

        session['heading'] += yaw_rate * dt
        dx = speed * math.cos(session['heading']) * dt
        dy = speed * math.sin(session['heading']) * dt
        session['x'] += dx
        session['y'] += dy

        inner_wheel_deg = steering['inner_wheel_angle'] if steering else 0.0
        outer_wheel_deg = steering['outer_wheel_angle'] if steering else 0.0
        lateral_accel = speed * yaw_rate if R_eff != float('inf') else 0.0

        m = config.dynamics.mass + (cargo.mass if cargo else 0)
        roll_stiff = config.dynamics.roll_stiffness
        damping = config.dynamics.damping_ratio * 2 * math.sqrt(m * roll_stiff)
        h_cg = config.dynamics.cg_height + (cargo.offset_height if cargo else 0)
        roll_moment = m * abs(lateral_accel) * h_cg
        roll_angle_rad = roll_moment / (roll_stiff + 1e-6)
        roll_angle_rad = min(math.radians(40), roll_angle_rad)
        if yaw_rate < 0:
            roll_angle_rad = -roll_angle_rad
        roll_rate = (roll_angle_rad - session.get('prev_roll', 0)) / dt
        session['prev_roll'] = roll_angle_rad

        if cargo and cargo.mass > 0 and cargo.shift_dynamics:
            F_lat = cargo.mass * lateral_accel + random.uniform(-0.5, 0.5) * bump_amp * 10
            a_lat = (F_lat - cargo.shift_stiffness * session['cargo_shift_lateral']
                     - cargo.shift_damping * session['cargo_velocity_lat']) / cargo.mass
            session['cargo_velocity_lat'] += a_lat * dt
            session['cargo_shift_lateral'] += session['cargo_velocity_lat'] * dt
            session['cargo_shift_lateral'] = max(-0.3, min(0.3, session['cargo_shift_lateral']))

            F_ver = -cargo.mass * lateral_accel * 0.3 + random.uniform(-0.3, 0.3) * bump_amp * 8
            a_ver = (F_ver - cargo.shift_stiffness * session['cargo_shift_vertical']
                     - cargo.shift_damping * session['cargo_velocity_ver']) / cargo.mass
            session['cargo_velocity_ver'] += a_ver * dt
            session['cargo_shift_vertical'] += session['cargo_velocity_ver'] * dt
            session['cargo_shift_vertical'] = max(-0.1, min(0.1, session['cargo_shift_vertical']))

        effective_cg_lat = config.dynamics.cg_lateral
        effective_cg_h = config.dynamics.cg_height
        if cargo and cargo.mass > 0:
            total_mass = config.dynamics.mass + cargo.mass
            y_cargo = cargo.offset_lateral + session['cargo_shift_lateral']
            effective_cg_lat = (config.dynamics.mass * config.dynamics.cg_lateral + cargo.mass * y_cargo) / total_mass
            h_cargo = config.dynamics.cg_height + cargo.offset_height + session['cargo_shift_vertical']
            effective_cg_h = (config.dynamics.mass * config.dynamics.cg_height + cargo.mass * h_cargo) / total_mass

        ssf = config.dynamics.track_width / (2 * max(0.01, effective_cg_h))
        roll_threshold = 9.81 * config.dynamics.track_width / (2 * max(0.01, effective_cg_h - config.dynamics.roll_center_height))
        a_y_norm = abs(lateral_accel) / 9.81
        risk_ratio = a_y_norm / max(0.1, roll_threshold / 9.81)
        cg_factor = 1.0 + 1.5 * abs(effective_cg_lat) / max(0.01, config.dynamics.track_width)
        mass_factor = 1.0
        if cargo and cargo.mass > 0:
            mass_factor = 1.0 + 0.1 * cargo.mass / config.dynamics.mass
        rollover_risk_pct = min(100, risk_ratio * cg_factor * mass_factor * 100)

        slip_ratio = 0.0
        if mu > 0 and abs(lateral_accel) > mu * 9.81 * 0.7:
            slip_ratio = min(1.0, (abs(lateral_accel) - mu * 9.81 * 0.7) / (mu * 9.81 * 0.3 + 0.01))

        wheel_r = config.geometry.wheel_radius
        if wheel_r > 0 and speed > 0:
            omega = speed / wheel_r
            for i in range(4):
                session['wheel_rotations'][i] += omega * dt
                if i < 2 and R_eff != float('inf') and abs(R_eff) > 0.5:
                    track = config.dynamics.track_width
                    r_wheel = R_eff - (-1 if i in (0, 2) else 1) * track / 2
                    session['wheel_rotations'][i] = session['wheel_rotations'][0] * r_wheel / R_eff if R_eff != 0 else 0

        is_tipping = rollover_risk_pct > 85
        is_stuck = mu < 0.25 and throttle > 0.5 and session['speed'] < 0.5
        alert_msg = ""
        if rollover_risk_pct > 70:
            alert_msg = "⚠ 侧翻风险高！请减速或回正方向"
        elif is_stuck:
            alert_msg = "⚠ 车轮打滑陷入泥地，请减少牵引力"
        elif slip_ratio > 0.5:
            alert_msg = "⚠ 严重侧滑，请小心操控"
        elif mu < 0.35:
            alert_msg = "路面湿滑，注意安全"

        stability_idx = max(0.0, 1.0 - rollover_risk_pct / 100.0)
        critical_v = math.sqrt(max(0.1, roll_threshold * R_eff)) if R_eff != float('inf') and R_eff > 0 else config.max_speed_mps

        return VirtualDriveState(
            session_id=session_id,
            vehicle_type=vehicle_type,
            road_type=road_type,
            x=session['x'],
            y=session['y'],
            heading=session['heading'],
            speed=session['speed'],
            pole_angle=pole_angle_deg,
            inner_wheel_angle=inner_wheel_deg,
            outer_wheel_angle=outer_wheel_deg,
            turning_radius=R_eff,
            roll_angle=math.degrees(roll_angle_rad),
            roll_rate=math.degrees(roll_rate),
            yaw_rate=math.degrees(yaw_rate),
            lateral_acceleration=lateral_accel,
            rollover_risk=rollover_risk_pct,
            stability_index=stability_idx,
            effective_friction=mu,
            slip_ratio=slip_ratio,
            wheel_rotation=list(session['wheel_rotations']),
            cargo_shift_lateral=session['cargo_shift_lateral'],
            cargo_shift_vertical=session['cargo_shift_vertical'],
            alert_message=alert_msg,
            is_tipping=is_tipping,
            is_stuck=is_stuck
        )

    def reset_session(self, session_id: str):
        if session_id in self._sessions:
            del self._sessions[session_id]
