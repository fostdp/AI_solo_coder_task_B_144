# -*- coding: utf-8 -*-
"""
era_comparator.py
=================
跨时代对比模块（独立后端模块 #2）

职责：古代车辆 vs 现代汽车 技术代际对比分析
- 不足转向梯度 K_us、临界车速、SSF、动力系统差异量化
- 自动生成研究洞察（古代畜力→现代内燃机 进化分析）
- 可独立导入：from common.era_comparator import EraComparator
"""
from typing import List, Dict, Any, Optional

from .steering_comparator import SteeringComparator
from .extended_models import MultiVehicleSteeringModel, CargoConfig
from .message_protocol import ComparisonResult


class EraComparator:
    """跨时代转向性能对比分析器"""

    def __init__(self):
        self._steering = MultiVehicleSteeringModel()
        self._steering_cmp = SteeringComparator()

    ANCIENT_DEFAULTS = ['chariot_double', 'wheelbarrow_single', 'chariot_four_wheel']
    MODERN_DEFAULTS = ['modern_car']

    def compare_eras(self, ancient_types: Optional[List[str]] = None,
                     modern_types: Optional[List[str]] = None,
                     pole_angle_deg: float = 20.0,
                     speed_mps: float = 5.0,
                     friction_coeff: float = 0.7,
                     road_type: str = 'ancient_post_road',
                     cargo: Optional[CargoConfig] = None) -> ComparisonResult:
        """
        执行古代 vs 现代的跨时代对比
        """
        if ancient_types is None:
            ancient = self.ANCIENT_DEFAULTS
        else:
            ancient = ancient_types
        if modern_types is None:
            modern = self.MODERN_DEFAULTS
        else:
            modern = modern_types
        all_vehicles = ancient + modern

        base = self._steering_cmp.compare(
            all_vehicles, pole_angle_deg, speed_mps, friction_coeff, road_type, cargo
        )
        ancient_entries = [e for e in base.entries if e.get('era') == '古代']
        modern_entries = [e for e in base.entries if e.get('era') == '现代']

        winners = dict(base.winners)
        winners.update(self._era_winners(ancient_entries, modern_entries))

        insights = list(base.insights)
        insights.extend(self._era_insights(ancient_entries, modern_entries,
                                           pole_angle_deg, speed_mps, road_type))

        return ComparisonResult(
            comparison_type='era_cross_generation',
            title='古代车辆与现代汽车跨时代转向性能对比',
            subtitle=f'古代 {len(ancient)}款 vs 现代 {len(modern)}款 | '
                     f'辕杆角 {pole_angle_deg}° | 车速 {speed_mps} m/s | 路面 {road_type}',
            input_conditions=dict(base.input_conditions,
                                  ancient_vehicles=ancient,
                                  modern_vehicles=modern),
            entries=base.entries,
            winners=winners,
            insights=insights
        )

    def _era_winners(self, ancient: List[Dict], modern: List[Dict]) -> Dict[str, str]:
        winners = {}
        if not ancient or not modern:
            return winners
        metrics = [
            ('古代最小转弯半径', 'turning_radius', True, ancient),
            ('现代最小转弯半径', 'turning_radius', True, modern),
            ('古代最高临界车速', 'critical_speed', False, ancient),
            ('现代最高临界车速', 'critical_speed', False, modern),
            ('古代最高SSF', 'ssf_static', False, ancient),
            ('现代最高SSF', 'ssf_static', False, modern),
        ]
        for name, key, lower_better, pool in metrics:
            if not pool:
                continue
            s = sorted(pool,
                       key=lambda e: e.get(key, float('inf') if lower_better else 0),
                       reverse=not lower_better)
            winners[name] = s[0]['vehicle_name']
        return winners

    def _era_insights(self, ancient: List[Dict], modern: List[Dict],
                      pole_deg: float, speed: float, road: str) -> List[str]:
        insights = []
        if not ancient or not modern:
            return insights
        avg_k_ancient = sum(e.get('understeer_gradient', 0) for e in ancient) / len(ancient)
        avg_k_modern = sum(e.get('understeer_gradient', 0) for e in modern) / len(modern)
        if abs(avg_k_ancient - avg_k_modern) > 0.01:
            better = '古代' if avg_k_ancient < avg_k_modern else '现代'
            insights.append(
                f"不足转向梯度 K_us：古代平均{avg_k_ancient:.3f}deg/g，"
                f"现代平均{avg_k_modern:.3f}deg/g，{better}车辆转向中性度更佳。"
            )
        max_v_ancient = max((e['max_speed_mps'] for e in ancient), default=0)
        max_v_modern = max((e['max_speed_mps'] for e in modern), default=0)
        if max_v_ancient > 0 and max_v_modern > max_v_ancient:
            ratio = max_v_modern / max_v_ancient
            insights.append(
                f"动力系统进化：古代畜力最高约{max_v_ancient * 3.6:.0f}km/h，"
                f"现代内燃机约{max_v_modern * 3.6:.0f}km/h，速度提升约{ratio:.1f}倍，"
                f"对应能量密度从畜力≈1kW/吨提升到内燃机≈60kW/吨。"
            )
        avg_ssf_a = sum(e['ssf_static'] for e in ancient) / len(ancient)
        avg_ssf_m = sum(e['ssf_static'] for e in modern) / len(modern)
        if avg_ssf_m > avg_ssf_a:
            insights.append(
                f"侧翻安全性：现代汽车平均SSF={avg_ssf_m:.2f}，"
                f"古代车辆平均SSF={avg_ssf_a:.2f}，"
                f"现代低重心设计使侧翻阈值提升{(avg_ssf_m / max(0.01, avg_ssf_a) - 1) * 100:.0f}%。"
            )
        insights.append(
            f"转向机构进化路径：四杆机构(秦汉)→前桥整体转向(汉辎车)→"
            f"蜗杆滚轮(1920s)→循环球(1950s)→齿轮齿条(1980s至今)，"
            f"阿克曼误差从约5%下降到<1.5%。"
        )
        return insights
