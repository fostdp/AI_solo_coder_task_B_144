# -*- coding: utf-8 -*-
"""
road_simulator.py
=================
路面影响仿真模块（独立后端模块 #3）

职责：不同路面条件对车辆操控稳定性影响分析
- 摩擦系数衰减侧偏刚度 / 滚动阻力 / 滑移 / 颠簸振动
- 多路面对比：干燥柏油/石板/土路/泥泞/碎石/沙土/冰雪/驿道
- 可独立导入：from common.road_simulator import RoadSimulator
"""
from typing import List, Dict, Any, Optional

from .extended_models import (
    RoadSurfaceModel, MultiVehicleSteeringModel, CargoConfig
)
from .message_protocol import RoadComparisonEntry, ComparisonResult


class RoadSimulator:
    """路面条件操控稳定性仿真器"""

    DEFAULT_ROADS = [
        'asphalt_dry', 'stone_pavement', 'dirt_road', 'mud_road',
        'gravel_road', 'sand_road', 'ice_snow', 'ancient_post_road'
    ]

    def __init__(self):
        self._road = RoadSurfaceModel()
        self._steering = MultiVehicleSteeringModel()

    def list_road_types(self) -> List[Dict[str, Any]]:
        return self._road.list_road_types()

    def compare_roads(self, vehicle_type: str, pole_angle_deg: float,
                      speed_mps: float, road_types: Optional[List[str]] = None,
                      cargo: Optional[CargoConfig] = None) -> ComparisonResult:
        if road_types is None:
            roads = self.DEFAULT_ROADS
        else:
            roads = road_types
        entries = []
        for rt in roads:
            cfg = self._road.get_surface_config(rt)
            if not cfg:
                continue
            mu = (cfg['friction_min'] + cfg['friction_max']) / 2
            steering = self._steering.compute_steering(
                vehicle_type, pole_angle_deg, speed_mps, mu, rt
            )
            stability = self._steering.compute_stability(
                vehicle_type, pole_angle_deg, speed_mps, 0.0, mu, cargo, rt
            )
            if not steering or not stability:
                continue
            veh_cfg = self._steering.get_vehicle_config(vehicle_type)
            v_eff = speed_mps / cfg.get('slip_factor', 1.0)
            traction = cfg.get('rolling_resistance', 0.05) * veh_cfg.dynamics.mass * 9.81
            entries.append(RoadComparisonEntry(
                road_type=rt,
                road_name=cfg.get('name', rt),
                category=cfg.get('category', ''),
                friction_coeff=mu,
                rolling_resistance=cfg.get('rolling_resistance', 0),
                slip_factor=cfg.get('slip_factor', 1.0),
                effective_speed=v_eff,
                turning_radius_effective=steering['turning_radius'] * cfg.get('slip_factor', 1.0),
                yaw_rate=stability['yaw_rate'],
                lateral_acceleration=stability['lateral_acceleration'],
                rollover_risk=stability['rollover_risk'],
                stability_index=stability['stability_index'],
                critical_speed=stability['critical_speed'],
                ackermann_error=steering['ackermann_error'],
                max_safe_speed=min(stability['critical_speed'], veh_cfg.max_speed_mps),
                traction_force_required=traction,
                vibration_level=stability['vibration_level']
            ).to_dict())
        winners = self._find_winners(entries)
        insights = self._generate_insights(entries, vehicle_type, pole_angle_deg, speed_mps)
        return ComparisonResult(
            comparison_type='road_surface',
            title='不同路面条件操控稳定性对比分析',
            subtitle=f'车辆: {self._steering.get_vehicle_config(vehicle_type).name if self._steering.get_vehicle_config(vehicle_type) else vehicle_type} | '
                     f'辕杆角 {pole_angle_deg}° | 车速 {speed_mps} m/s',
            input_conditions={
                'vehicle_type': vehicle_type,
                'pole_angle_deg': pole_angle_deg,
                'speed_mps': speed_mps,
                'road_types': roads
            },
            entries=entries,
            winners=winners,
            insights=insights
        )

    def compute_road_effects(self, road_type: str, vehicle_dynamics=None) -> Dict[str, Any]:
        cfg = self._road.get_surface_config(road_type)
        mu = (cfg['friction_min'] + cfg['friction_max']) / 2
        slip_factor = cfg['slip_factor']
        rolling_resistance = cfg['rolling_resistance']
        bump_amp = cfg['bump_amplitude_m']
        irregularity = cfg['irregularity']
        vibration = irregularity * 2.0 + bump_amp * 10.0
        result = {
            'road_type': road_type,
            'road_name': cfg.get('name', road_type),
            'friction_coeff': mu,
            'rolling_resistance': rolling_resistance,
            'slip_factor': slip_factor,
            'bump_amplitude': bump_amp,
            'irregularity': irregularity,
            'vibration_acceleration': vibration
        }
        if vehicle_dynamics is not None:
            c_friction = max(0.2, min(1.0, mu / 0.85))
            c_f = vehicle_dynamics.cornering_stiffness_front * c_friction
            c_r = vehicle_dynamics.cornering_stiffness_rear * c_friction
            result['effective_cornering_stiffness_front'] = c_f
            result['effective_cornering_stiffness_rear'] = c_r
        return result

    def _find_winners(self, entries: List[Dict]) -> Dict[str, str]:
        winners = {}
        if not entries:
            return winners
        cats = [
            ('最高摩擦系数', 'friction_coeff', False),
            ('最低滚动阻力', 'rolling_resistance', True),
            ('最低侧翻风险', 'rollover_risk', True),
            ('最高稳定性指数', 'stability_index', False),
            ('最高安全车速', 'max_safe_speed', False),
            ('最小有效转弯半径', 'turning_radius_effective', True),
            ('最低牵引力需求', 'traction_force_required', True),
            ('最低颠簸振动', 'vibration_level', True)
        ]
        for name, key, lower_better in cats:
            s = sorted(entries,
                       key=lambda e: e.get(key, float('inf') if lower_better else 0),
                       reverse=not lower_better)
            winners[name] = s[0]['road_name']
        return winners

    def _generate_insights(self, entries: List[Dict], vehicle_type: str,
                           pole_deg: float, speed: float) -> List[str]:
        insights = []
        if not entries:
            return insights
        best = min(entries, key=lambda e: e.get('rollover_risk', 100))
        worst = max(entries, key=lambda e: e.get('rollover_risk', 0))
        insights.append(
            f"最佳路面【{best['road_name']}】侧翻风险仅{best['rollover_risk']:.0f}%，"
            f"最差【{worst['road_name']}】高达{worst['rollover_risk']:.0f}%，"
            f"相差{worst['rollover_risk'] - best['rollover_risk']:.0f}个百分点。"
        )
        if worst.get('rollover_risk', 0) > 70 and speed > 3:
            safe_v = max(1, int(worst.get('max_safe_speed', 2)))
            insights.append(
                f"⚠ 在【{worst['road_name']}】上以{speed}m/s行驶存在严重危险，"
                f"建议将车速降至{safe_v}m/s以下。"
            )
        paved = [e for e in entries if '铺装' in e.get('category', '')]
        unpaved = [e for e in entries if '非铺装' in e.get('category', '')]
        if paved and unpaved:
            avg_sp = sum(e['stability_index'] for e in paved) / len(paved)
            avg_su = sum(e['stability_index'] for e in unpaved) / len(unpaved)
            insights.append(
                f"铺装路面平均稳定性({avg_sp:.2f})优于非铺装({avg_su:.2f})，"
                f"秦汉驿道系统建设显著提升了运输效率。"
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
