# -*- coding: utf-8 -*-
"""
vr_chariot.py
=============
虚拟驾驶双辕车引擎（独立后端模块 #4）

职责：公众虚拟驾驶双辕车体验
- 用户辕杆/方向盘输入 → 单步物理仿真（位置/航向/速度/横摆/侧倾）
- 货物弹簧-阻尼位移模型
- 陷泥检测、侧翻告警
- 完整力反馈方向盘模型（回正+阻尼+路感+摩擦，ISO 11663）
- 可独立导入：from common.vr_chariot import VRChariotEngine
"""
import math
import random
import time
from typing import Dict, Any, Optional

from .extended_models import (
    MultiVehicleSteeringModel, RoadSurfaceModel, ForceFeedbackModel
)
from .stability_analysis import CargoConfig
from .message_protocol import VirtualDriveState


class VRChariotEngine:
    """虚拟驾驶双辕车物理仿真引擎"""

    def __init__(self):
        self._steering = MultiVehicleSteeringModel()
        self._road = RoadSurfaceModel()
        self._ffb = ForceFeedbackModel()
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
                'prev_roll': 0.0,
                'cargo_shift_lateral': 0.0,
                'cargo_shift_vertical': 0.0,
                'cargo_velocity_lat': 0.0,
                'cargo_velocity_ver': 0.0
            }
        return self._sessions[session_id]

    def step(self, session_id: str, vehicle_type: str, road_type: str,
             pole_angle_deg: float, throttle: float, brake: float,
             cargo: Optional[CargoConfig] = None,
             dt: float = 0.05) -> VirtualDriveState:
        """
        执行一步物理仿真 (dt秒)
        """
        session = self._get_or_create_session(session_id)
        cfg = self._steering.get_vehicle_config(vehicle_type)
        if not cfg:
            cfg = self._steering.get_vehicle_config('chariot_double')
        road_cfg = self._road.get_surface_config(road_type)
        mu = (road_cfg['friction_min'] + road_cfg['friction_max']) / 2
        roll_resist = road_cfg.get('rolling_resistance', 0.05)
        slip_factor = road_cfg.get('slip_factor', 1.0)
        bump_amp = road_cfg.get('bump_amplitude_m', 0.02)

        accel_target = throttle * 2.0
        brake_force = brake * 5.0
        rolling_force = roll_resist * 9.81
        session['speed'] += (accel_target - rolling_force - brake_force) * dt
        session['speed'] = max(0.0, min(session['speed'], cfg.max_speed_mps))
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

        inner_deg = steering['inner_wheel_angle'] if steering else 0.0
        outer_deg = steering['outer_wheel_angle'] if steering else 0.0
        lateral_accel = speed * yaw_rate if R_eff != float('inf') else 0.0

        m = cfg.dynamics.mass + (cargo.mass if cargo else 0)
        roll_stiff = cfg.dynamics.roll_stiffness
        damp = cfg.dynamics.damping_ratio * 2 * math.sqrt(m * roll_stiff)
        h_cg = cfg.dynamics.cg_height + (cargo.offset_height if cargo else 0)
        roll_moment = m * abs(lateral_accel) * h_cg
        roll_rad = roll_moment / (roll_stiff + 1e-6)
        roll_rad = min(math.radians(40), roll_rad)
        if yaw_rate < 0:
            roll_rad = -roll_rad
        roll_rate = (roll_rad - session.get('prev_roll', 0)) / dt
        session['prev_roll'] = roll_rad

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

        eff_cg_lat = cfg.dynamics.cg_lateral
        eff_cg_h = cfg.dynamics.cg_height
        if cargo and cargo.mass > 0:
            total = cfg.dynamics.mass + cargo.mass
            y_cargo = cargo.offset_lateral + session['cargo_shift_lateral']
            eff_cg_lat = (cfg.dynamics.mass * cfg.dynamics.cg_lateral + cargo.mass * y_cargo) / total
            h_cargo = cfg.dynamics.cg_height + cargo.offset_height + session['cargo_shift_vertical']
            eff_cg_h = (cfg.dynamics.mass * cfg.dynamics.cg_height + cargo.mass * h_cargo) / total

        ssf = cfg.dynamics.track_width / (2 * max(0.01, eff_cg_h))
        roll_thresh = 9.81 * cfg.dynamics.track_width / (2 * max(0.01, eff_cg_h - cfg.dynamics.roll_center_height))
        a_y_norm = abs(lateral_accel) / 9.81
        risk_ratio = a_y_norm / max(0.1, roll_thresh / 9.81)
        cg_factor = 1.0 + 1.5 * abs(eff_cg_lat) / max(0.01, cfg.dynamics.track_width)
        mass_factor = 1.0
        if cargo and cargo.mass > 0:
            mass_factor = 1.0 + 0.1 * cargo.mass / cfg.dynamics.mass
        rollover_risk_pct = min(100, risk_ratio * cg_factor * mass_factor * 100)

        slip_ratio = 0.0
        if mu > 0 and abs(lateral_accel) > mu * 9.81 * 0.7:
            slip_ratio = min(1.0, (abs(lateral_accel) - mu * 9.81 * 0.7) / (mu * 9.81 * 0.3 + 0.01))

        wheel_r = cfg.geometry.wheel_radius
        if wheel_r > 0 and speed > 0:
            omega = speed / wheel_r
            for i in range(4):
                session['wheel_rotations'][i] += omega * dt
                if i < 2 and R_eff != float('inf') and abs(R_eff) > 0.5:
                    trk = cfg.dynamics.track_width
                    r_wheel = R_eff - (-1 if i in (0, 2) else 1) * trk / 2
                    session['wheel_rotations'][i] = session['wheel_rotations'][0] * r_wheel / R_eff if R_eff != 0 else 0

        is_tipping = rollover_risk_pct > 85
        is_stuck = mu < 0.25 and throttle > 0.5 and session['speed'] < 0.5
        alert = ""
        if rollover_risk_pct > 70:
            alert = "⚠ 侧翻风险高！请减速或回正方向"
        elif is_stuck:
            alert = "⚠ 车轮打滑陷入泥地，请减少牵引力"
        elif slip_ratio > 0.5:
            alert = "⚠ 严重侧滑，请小心操控"
        elif mu < 0.35:
            alert = "路面湿滑，注意安全"

        stability_idx = max(0.0, 1.0 - rollover_risk_pct / 100.0)
        critical_v = math.sqrt(max(0.1, roll_thresh * R_eff)) if R_eff != float('inf') and R_eff > 0 else cfg.max_speed_mps

        road_effect = self._road.compute_effects(road_type, cfg.dynamics, mu)
        wheel_avg_rad = math.radians((inner_deg + outer_deg) / 2.0)
        ffb = self._ffb.compute(
            vehicle_dynamics=cfg.dynamics,
            road_cfg=road_cfg,
            pole_angle_deg=pole_angle_deg,
            wheel_angle_avg_rad=wheel_avg_rad,
            speed_mps=speed,
            lateral_accel_mps2=lateral_accel,
            slip_ratio=slip_ratio,
            mu=mu,
            cornering_stiff_front_N_per_rad=road_effect.effective_cornering_stiffness_front,
            dt=dt
        )

        return VirtualDriveState(
            session_id=session_id,
            vehicle_type=vehicle_type,
            road_type=road_type,
            x=session['x'],
            y=session['y'],
            heading=session['heading'],
            speed=session['speed'],
            pole_angle=pole_angle_deg,
            inner_wheel_angle=inner_deg,
            outer_wheel_angle=outer_deg,
            turning_radius=R_eff,
            roll_angle=math.degrees(roll_rad),
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
            ffb_total_torque=ffb.get('ffb_total_torque', 0.0),
            ffb_aligning_torque=ffb.get('ffb_aligning_torque', 0.0),
            ffb_damping_torque=ffb.get('ffb_damping_torque', 0.0),
            ffb_road_feel_torque=ffb.get('ffb_road_feel_torque', 0.0),
            ffb_friction_torque=ffb.get('ffb_friction_torque', 0.0),
            ffb_intensity=ffb.get('ffb_intensity', 0.0),
            alert_message=alert,
            is_tipping=is_tipping,
            is_stuck=is_stuck
        )

    def reset_session(self, session_id: str) -> None:
        if session_id in self._sessions:
            del self._sessions[session_id]

    def list_sessions(self):
        return list(self._sessions.keys())
