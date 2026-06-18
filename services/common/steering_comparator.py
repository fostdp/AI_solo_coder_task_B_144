# -*- coding: utf-8 -*-
"""
steering_comparator.py
======================
转向机构对比模块（独立后端模块 #1）

职责：多车型转向机构几何与动力学对比分析
- 不同车型(独轮/双辕/四轮/现代)转向半径、阿克曼误差、传动角对比
- 输出胜者/洞察/结构化表格数据
- 可独立导入测试：from common.steering_comparator import SteeringComparator

架构：纯函数 + 类封装，无副作用，依赖 extended_models MultiVehicleSteeringModel
"""
import math
from typing import List, Dict, Any, Optional

from .extended_models import MultiVehicleSteeringModel, CargoConfig
from .message_protocol import VehicleComparisonEntry, ComparisonResult


class SteeringComparator:
    """转向机构对比分析器"""

    def __init__(self):
        self._steering = MultiVehicleSteeringModel()

    def compare(self, vehicle_types: List[str], pole_angle_deg: float,
                speed_mps: float, friction_coeff: float,
                road_type: str = 'dirt_road',
                cargo: Optional[CargoConfig] = None) -> ComparisonResult:
        """
        执行多车型转向机构对比
        :return: ComparisonResult 含 entries/winners/insights
        """
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
            cfg = self._steering.get_vehicle_config(vt)
            entries.append(VehicleComparisonEntry(
                vehicle_type=vt,
                vehicle_name=steering['vehicle_name'],
                era=cfg.era if cfg else '未知',
                category=cfg.category if cfg else '未知',
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
        winners = self._find_winners(entries)
        insights = self._generate_insights(entries, pole_angle_deg, speed_mps, road_type)
        return ComparisonResult(
            comparison_type='vehicle_steering',
            title='多车型转向机构对比分析',
            subtitle=f'辕杆角 {pole_angle_deg}° | 车速 {speed_mps} m/s | 路面: {road_type}',
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
        cfg = self._steering.get_vehicle_config(vehicle_type)
        if not cfg:
            return 999.0
        max_delta = math.radians(cfg.max_steering_angle_deg)
        if abs(max_delta) < 0.01:
            return float('inf')
        return cfg.geometry.wheelbase / math.tan(max_delta * 0.85)

    def _find_winners(self, entries: List[Dict]) -> Dict[str, str]:
        winners = {}
        if not entries:
            return winners
        categories = [
            ('最小转弯半径', 'min_turning_radius', True),
            ('最高临界速度', 'critical_speed', False),
            ('最低阿克曼误差', 'ackermann_error', True),
            ('最高稳定性指数', 'stability_index', False),
            ('最低侧翻风险', 'rollover_risk', True),
            ('最高SSF', 'ssf_static', False),
            ('最大内轮转角', 'max_inner_wheel_angle', False),
            ('最高车速', 'max_speed_mps', False)
        ]
        for name, key, lower_better in categories:
            s = sorted(entries,
                       key=lambda e: e.get(key, float('inf') if lower_better else 0),
                       reverse=not lower_better)
            winners[name] = s[0]['vehicle_name']
        return winners

    def _generate_insights(self, entries: List[Dict], pole_deg: float,
                           speed: float, road_type: str) -> List[str]:
        insights = []
        if len(entries) < 2:
            return insights
        ancient = [e for e in entries if e.get('era') == '古代']
        modern = [e for e in entries if e.get('era') == '现代']
        if ancient and modern:
            avg_r_a = sum(e['turning_radius'] for e in ancient
                          if e['turning_radius'] != float('inf')) / max(1, len(ancient))
            avg_r_m = sum(e['turning_radius'] for e in modern
                          if e['turning_radius'] != float('inf')) / max(1, len(modern))
            if avg_r_a < avg_r_m:
                insights.append(
                    f"古代车辆平均转弯半径({avg_r_a:.1f}m)比现代汽车({avg_r_m:.1f}m)更紧凑，适配古代狭窄街道。"
                )
            else:
                insights.append(
                    f"现代汽车平均转弯半径({avg_r_m:.1f}m)优于古代车辆({avg_r_a:.1f}m)，齿轮齿条转向精度更高。"
                )
            avg_s_a = sum(e['stability_index'] for e in ancient) / len(ancient)
            avg_s_m = sum(e['stability_index'] for e in modern) / len(modern)
            insights.append(
                f"现代汽车稳定性指数({avg_s_m:.2f})比古代车辆({avg_s_a:.2f})高出"
                f"{((avg_s_m - avg_s_a) / max(0.01, avg_s_a) * 100):.0f}%，低重心+独立悬架贡献显著。"
            )
            v_a = max((e['max_speed_mps'] for e in ancient), default=0)
            v_m = max((e['max_speed_mps'] for e in modern), default=0)
            insights.append(
                f"动力差异：古代畜力最高约{v_a * 3.6:.0f}km/h，"
                f"现代内燃机可达{v_m * 3.6:.0f}km/h，速度提升约{(v_m / max(0.1, v_a)):.1f}倍。"
            )
        for e in entries:
            if e.get('ackermann_error', 0) > 0.05:
                insights.append(
                    f"{e['vehicle_name']}阿克曼误差{e['ackermann_error'] * 100:.1f}%，高速转弯轮胎磨损明显。"
                )
            if e.get('rollover_risk', 0) > 50:
                insights.append(
                    f"⚠ {e['vehicle_name']}侧翻风险{e['rollover_risk']:.0f}%，重心偏高是主因。"
                )
        wheelbarrow = [e for e in entries if e.get('category') == '独轮车']
        if wheelbarrow:
            w = wheelbarrow[0]
            r = w['turning_radius'] if w['turning_radius'] != float('inf') else '∞'
            insights.append(
                f"独轮车转弯半径仅{r}m，原地转向极强，但侧翻风险{w['rollover_risk']:.0f}%，需极高驾驶技巧。"
            )
        if not insights:
            insights.append("各车型当前工况表现均衡，可调整参数观察差异。")
        return insights
