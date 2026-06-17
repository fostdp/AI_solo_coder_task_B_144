import math
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class VehicleDynamicsParams:
    wheelbase: float = 2.5
    track_width: float = 1.8
    cg_height: float = 0.8
    cg_longitudinal: float = 0.0
    cg_lateral: float = 0.0
    roll_center_height: float = 0.3
    mass: float = 800.0
    yaw_inertia: float = 1200.0
    roll_stiffness: float = 30000.0
    damping_ratio: float = 0.3
    wheel_radius: float = 0.35
    cornering_stiffness_front: float = 25000.0
    cornering_stiffness_rear: float = 35000.0


@dataclass
class CargoConfig:
    mass: float = 0.0
    offset_lateral: float = 0.0
    offset_longitudinal: float = 0.0
    offset_height: float = 0.0
    shift_dynamics: bool = True
    shift_stiffness: float = 5000.0
    shift_damping: float = 800.0


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


class VariableCGModel:
    def __init__(self, vehicle_params: VehicleDynamicsParams, cargo: CargoConfig = None):
        self.vp = vehicle_params
        self.cargo = cargo or CargoConfig()
        self._cargo_dyn_lateral = 0.0
        self._cargo_dyn_vertical = 0.0
        self._cargo_dyn_velocity_lat = 0.0
        self._cargo_dyn_velocity_ver = 0.0

    def compute_effective_cg(self) -> Tuple[float, float, float]:
        m_body = self.vp.mass
        m_cargo = self.cargo.mass
        m_total = m_body + m_cargo

        if m_cargo <= 0:
            return (self.vp.cg_height, self.vp.cg_lateral, self.vp.cg_longitudinal)

        total_shift_lat = self.cargo.offset_lateral + self._cargo_dyn_lateral
        total_shift_ver = self.cargo.offset_height + self._cargo_dyn_vertical
        total_shift_lon = self.cargo.offset_longitudinal

        h_cg_eff = (m_body * self.vp.cg_height +
                    m_cargo * (self.vp.cg_height + total_shift_ver)) / m_total

        y_cg_eff = (m_body * self.vp.cg_lateral +
                    m_cargo * (self.vp.cg_lateral + total_shift_lat)) / m_total

        x_cg_eff = (m_body * self.vp.cg_longitudinal +
                    m_cargo * (self.vp.cg_longitudinal + total_shift_lon)) / m_total

        return (h_cg_eff, y_cg_eff, x_cg_eff)

    def compute_effective_yaw_inertia(self) -> float:
        m_cargo = self.cargo.mass
        I_base = self.vp.yaw_inertia

        if m_cargo <= 0:
            return I_base

        _, y_cg_eff, x_cg_eff = self.compute_effective_cg()

        cargo_abs_y = self.vp.cg_lateral + self.cargo.offset_lateral + self._cargo_dyn_lateral - y_cg_eff
        cargo_abs_x = self.vp.cg_longitudinal + self.cargo.offset_longitudinal - x_cg_eff

        I_parallel_axis = m_cargo * (cargo_abs_x ** 2 + cargo_abs_y ** 2)
        I_cargo_local = m_cargo * (0.3 ** 2 + 0.4 ** 2) / 12

        return I_base + I_parallel_axis + I_cargo_local

    def update_cargo_dynamics(self, lateral_accel: float, vertical_accel: float,
                              roll_angle_deg: float, dt: float = 60.0):
        if not self.cargo.shift_dynamics or self.cargo.mass <= 0:
            self._cargo_dyn_lateral = 0.0
            self._cargo_dyn_vertical = 0.0
            return (0.0, 0.0)

        dt_sec = min(dt, 1.0)
        m = self.cargo.mass
        k = self.cargo.shift_stiffness
        c = self.cargo.shift_damping

        roll_rad = math.radians(roll_angle_deg)
        lat_force = m * lateral_accel * math.cos(roll_rad) + m * 9.81 * math.sin(roll_rad)
        ver_force = -m * vertical_accel + m * 9.81 * (1 - math.cos(roll_rad))

        a_lat = (lat_force - k * self._cargo_dyn_lateral - c * self._cargo_dyn_velocity_lat) / m
        self._cargo_dyn_velocity_lat += a_lat * dt_sec
        self._cargo_dyn_lateral += self._cargo_dyn_velocity_lat * dt_sec

        a_ver = (ver_force - k * 0.3 * self._cargo_dyn_vertical - c * 0.3 * self._cargo_dyn_velocity_ver) / m
        self._cargo_dyn_velocity_ver += a_ver * dt_sec
        self._cargo_dyn_vertical += self._cargo_dyn_velocity_ver * dt_sec

        max_shift = 0.3
        self._cargo_dyn_lateral = max(-max_shift, min(max_shift, self._cargo_dyn_lateral))
        self._cargo_dyn_vertical = max(-0.1, min(0.1, self._cargo_dyn_vertical))

        return (self._cargo_dyn_lateral, self._cargo_dyn_vertical)

    def get_cargo_shift(self) -> Tuple[float, float]:
        return (self._cargo_dyn_lateral, self._cargo_dyn_vertical)

    def reset(self):
        self._cargo_dyn_lateral = 0.0
        self._cargo_dyn_vertical = 0.0
        self._cargo_dyn_velocity_lat = 0.0
        self._cargo_dyn_velocity_ver = 0.0


class RollCenterAnalyzer:
    def __init__(self, params: VehicleDynamicsParams = None):
        self.params = params or VehicleDynamicsParams()

    def calculate_roll_center_height(self, roll_angle_deg: float,
                                     wheel_displacement_left: float = 0.0,
                                     wheel_displacement_right: float = 0.0,
                                     cg_lateral_offset: float = 0.0) -> float:
        T = self.params.track_width
        h_rc_base = self.params.roll_center_height

        roll_angle = math.radians(roll_angle_deg)

        lateral_shift = h_rc_base * math.sin(roll_angle) + cg_lateral_offset * 0.2

        track_change = abs(wheel_displacement_left - wheel_displacement_right)
        h_rc_dynamic = h_rc_base * (1 + 0.1 * track_change / T)

        roll_geometry_effect = math.sin(abs(roll_angle)) * T / 2 * 0.15

        return h_rc_dynamic + roll_geometry_effect


class YawRateAnalyzer:
    def __init__(self, params: VehicleDynamicsParams = None,
                 variable_cg: VariableCGModel = None):
        self.params = params or VehicleDynamicsParams()
        self.vcg = variable_cg

    def _compute_weight_distribution(self, cg_longitudinal: float) -> Tuple[float, float]:
        L = self.params.wheelbase
        W_f = self.params.mass * 9.81 * (0.5 - cg_longitudinal / L)
        W_r = self.params.mass * 9.81 * (0.5 + cg_longitudinal / L)
        return max(W_f, 0.1), max(W_r, 0.1)

    def calculate_yaw_rate(self, speed: float, steering_angle_deg: float,
                           friction_coeff: float = 0.7,
                           lateral_accel_prev: float = 0.0,
                           dt: float = 60.0) -> float:
        if self.vcg:
            h_cg, y_cg, x_cg = self.vcg.compute_effective_cg()
            I_z = self.vcg.compute_effective_yaw_inertia()
        else:
            h_cg = self.params.cg_height
            y_cg = self.params.cg_lateral
            x_cg = self.params.cg_longitudinal
            I_z = self.params.yaw_inertia

        L = self.params.wheelbase
        T = self.params.track_width
        m = self.params.mass + (self.vcg.cargo.mass if self.vcg else 0)

        steering_angle = math.radians(steering_angle_deg)

        if abs(steering_angle) < 0.001:
            return 0.0

        W_f, W_r = self._compute_weight_distribution(x_cg)
        a = L / 2 + x_cg
        b = L / 2 - x_cg

        C_f = self.params.cornering_stiffness_front
        C_r = self.params.cornering_stiffness_rear

        if abs(speed) < 0.1:
            R_kinematic = L / math.tan(abs(steering_angle))
            return speed / R_kinematic * (1 if steering_angle > 0 else -1)

        u = speed
        K_us = (W_f / C_f - W_r / C_r) * 180 / math.pi

        understeer_factor = 1 + (K_us * m * u ** 2) / (L ** 2 * 9.81)
        if understeer_factor < 0.1:
            understeer_factor = 0.1

        yaw_rate_bicycle = (u / L * math.tan(steering_angle)) / understeer_factor

        if self.vcg and self.vcg.cargo.mass > 0:
            lateral_force = m * lateral_accel_prev
            yaw_moment_correction = lateral_force * y_cg
            yaw_accel = yaw_moment_correction / I_z if I_z > 0 else 0
            yaw_rate_dynamic = yaw_rate_bicycle + yaw_accel * min(dt, 1.0)
        else:
            yaw_rate_dynamic = yaw_rate_bicycle

        slip_factor = 1.0 - 0.3 * (1.0 - friction_coeff)
        cg_height_factor = 1.0 - 0.05 * (h_cg - 0.8) / 0.8
        actual_yaw_rate = yaw_rate_dynamic * slip_factor * cg_height_factor

        max_yaw_rate = 3.0
        actual_yaw_rate = max(-max_yaw_rate, min(max_yaw_rate, actual_yaw_rate))

        return actual_yaw_rate

    def calculate_lateral_acceleration(self, speed: float, yaw_rate: float,
                                       roll_angle_deg: float = 0.0,
                                       cg_height: float = None) -> float:
        h_cg = cg_height or self.params.cg_height

        ay_centripetal = speed * yaw_rate

        roll_angle = math.radians(roll_angle_deg)
        roll_induced_accel = 9.81 * math.sin(roll_angle) * (1 + h_cg / self.params.track_width)

        ay_corrected = ay_centripetal * math.cos(roll_angle) + roll_induced_accel

        return ay_corrected


class RolloverRiskAnalyzer:
    def __init__(self, params: VehicleDynamicsParams = None,
                 variable_cg: VariableCGModel = None):
        self.params = params or VehicleDynamicsParams()
        self.vcg = variable_cg
        self.roll_center_analyzer = RollCenterAnalyzer(params)
        self.yaw_analyzer = YawRateAnalyzer(params, variable_cg)

    def calculate_ssf(self, cg_height: float = None, cg_lateral: float = 0.0) -> float:
        T = self.params.track_width
        h = cg_height or self.params.cg_height
        effective_T = T - 2 * abs(cg_lateral)
        effective_T = max(effective_T, T * 0.3)
        return effective_T / (2 * h)

    def calculate_rollover_risk(self, speed: float, steering_angle_deg: float,
                                friction_coeff: float = 0.7,
                                roll_angle_deg: float = 0.0,
                                dt: float = 60.0) -> Tuple[float, str, dict]:
        if self.vcg:
            h_cg, y_cg, x_cg = self.vcg.compute_effective_cg()
            I_z = self.vcg.compute_effective_yaw_inertia()
        else:
            h_cg = self.params.cg_height
            y_cg = self.params.cg_lateral
            x_cg = self.params.cg_longitudinal
            I_z = self.params.yaw_inertia

        T = self.params.track_width

        h_rc = self.roll_center_analyzer.calculate_roll_center_height(
            roll_angle_deg, cg_lateral_offset=y_cg
        )
        h_eff = h_cg - h_rc
        h_eff = max(h_eff, 0.05)

        yaw_rate = self.yaw_analyzer.calculate_yaw_rate(
            speed, steering_angle_deg, friction_coeff, 0.0, dt
        )
        ay = self.yaw_analyzer.calculate_lateral_acceleration(
            speed, yaw_rate, roll_angle_deg, h_cg
        )

        ay_g = abs(ay) / 9.81

        ssf = self.calculate_ssf(h_cg, y_cg)
        roll_threshold = T / (2 * h_eff) if h_eff > 0 else float('inf')

        lateral_load_shift = ay_g * 2 * h_cg / T
        wheel_load_ratio = 0.5 + lateral_load_shift / 2
        wheel_load_ratio = max(0.0, min(1.0, wheel_load_ratio))

        inner_wheel_unloaded = wheel_load_ratio < 0.05

        risk_ratio_base = ay_g / roll_threshold if roll_threshold > 0 else 0

        cg_lateral_factor = 1.0 + 1.5 * abs(y_cg) / T
        cargo_mass = self.vcg.cargo.mass if self.vcg else 0
        mass_factor = 1.0 + 0.1 * cargo_mass / self.params.mass

        risk_ratio = risk_ratio_base * cg_lateral_factor * mass_factor

        if inner_wheel_unloaded:
            risk_ratio = max(risk_ratio, 0.7)

        risk_percentage = min(100.0, risk_ratio * 100)

        if risk_percentage < 30:
            level = "安全"
        elif risk_percentage < 60:
            level = "注意"
        elif risk_percentage < 85:
            level = "警告"
        else:
            level = "危险"

        details = {
            "ssf": ssf,
            "roll_threshold_g": roll_threshold,
            "lateral_accel_g": ay_g,
            "inner_wheel_load_ratio": wheel_load_ratio,
            "inner_wheel_unloaded": inner_wheel_unloaded,
            "effective_cg_height": h_cg,
            "effective_cg_lateral": y_cg,
            "effective_h_eff": h_eff,
            "yaw_inertia": I_z
        }

        return risk_percentage, level, details

    def calculate_critical_speed(self, friction_coeff: float = 0.7,
                                  steering_angle_deg: float = 10.0,
                                  cg_height: float = None,
                                  cg_lateral: float = 0.0) -> float:
        if self.vcg:
            h_cg, y_cg, _ = self.vcg.compute_effective_cg()
        else:
            h_cg = cg_height or self.params.cg_height
            y_cg = cg_lateral

        L = self.params.wheelbase
        T = self.params.track_width
        h_rc = self.params.roll_center_height

        steering_angle = math.radians(steering_angle_deg)

        R = L / math.tan(steering_angle) if abs(steering_angle) > 0.001 else 100

        effective_T = T - 2 * abs(y_cg)
        effective_T = max(effective_T, T * 0.3)

        v_rollover = math.sqrt(9.81 * effective_T * R / (2 * (h_cg - h_rc)))

        v_slide = math.sqrt(9.81 * friction_coeff * R)

        return min(v_rollover, v_slide)


class StabilityAnalyzer:
    def __init__(self, params: VehicleDynamicsParams = None, cargo: CargoConfig = None):
        self.params = params or VehicleDynamicsParams()
        self.cargo = cargo or CargoConfig()
        self.vcg = VariableCGModel(self.params, self.cargo)
        self.roll_center = RollCenterAnalyzer(params)
        self.yaw_analyzer = YawRateAnalyzer(params, self.vcg)
        self.rollover_risk = RolloverRiskAnalyzer(params, self.vcg)

    def set_cargo(self, cargo: CargoConfig):
        self.cargo = cargo
        self.vcg = VariableCGModel(self.params, self.cargo)
        self.yaw_analyzer.vcg = self.vcg
        self.rollover_risk.vcg = self.vcg

    def analyze(self, speed: float, pole_angle_deg: float, roll_angle_deg: float,
                slip_rate: float = 0.1, friction_coeff: float = 0.7,
                dt: float = 60.0, vertical_accel: float = 0.0) -> StabilityResult:
        h_cg, y_cg, x_cg = self.vcg.compute_effective_cg()

        ay_est = speed ** 2 * math.tan(math.radians(pole_angle_deg)) / self.params.wheelbase
        self.vcg.update_cargo_dynamics(ay_est, vertical_accel, roll_angle_deg, dt)

        h_cg, y_cg, x_cg = self.vcg.compute_effective_cg()
        I_z = self.vcg.compute_effective_yaw_inertia()
        cargo_shift_lat, cargo_shift_ver = self.vcg.get_cargo_shift()

        h_rc = self.roll_center.calculate_roll_center_height(
            roll_angle_deg, cg_lateral_offset=y_cg
        )

        yaw_rate = self.yaw_analyzer.calculate_yaw_rate(
            speed, pole_angle_deg, friction_coeff, ay_est, dt
        )

        ay = self.yaw_analyzer.calculate_lateral_acceleration(
            speed, yaw_rate, roll_angle_deg, h_cg
        )

        roll_rate = roll_angle_deg / dt if dt > 0 else 0

        risk_pct, _, details = self.rollover_risk.calculate_rollover_risk(
            speed, pole_angle_deg, friction_coeff, roll_angle_deg, dt
        )

        ay_g = abs(ay) / 9.81
        base_stability = max(0.0, min(1.0, 1.0 - risk_pct / 100.0))

        cargo_factor = 1.0
        if self.cargo.mass > 0:
            cargo_factor -= 0.15 * (self.cargo.mass / self.params.mass)
            cargo_factor -= 0.2 * abs(cargo_shift_lat) / 0.3

        stability_index = base_stability * max(0.4, cargo_factor)

        understeer_gradient = self._calculate_understeer_gradient(
            pole_angle_deg, ay, friction_coeff, x_cg
        )

        critical_speed = self.rollover_risk.calculate_critical_speed(
            friction_coeff,
            pole_angle_deg if abs(pole_angle_deg) > 1 else 10,
            h_cg, y_cg
        )

        return StabilityResult(
            roll_angle=roll_angle_deg,
            roll_rate=roll_rate,
            yaw_rate=math.degrees(yaw_rate),
            lateral_acceleration=ay,
            roll_center_height=h_rc,
            rollover_risk=risk_pct,
            stability_index=stability_index,
            understeer_gradient=understeer_gradient,
            critical_speed=critical_speed,
            effective_cg_height=h_cg,
            effective_cg_lateral=y_cg,
            effective_cg_longitudinal=x_cg,
            effective_yaw_inertia=I_z,
            cargo_shift_lateral=cargo_shift_lat,
            cargo_shift_vertical=cargo_shift_ver
        )

    def _calculate_understeer_gradient(self, steering_angle_deg: float,
                                       lateral_accel: float,
                                       friction_coeff: float,
                                       cg_longitudinal: float = 0.0) -> float:
        if abs(lateral_accel) < 0.01:
            return 0.0

        L = self.params.wheelbase
        m = self.params.mass + self.cargo.mass
        steering_angle = math.radians(steering_angle_deg)

        W_f, W_r = self._weight_distribution(cg_longitudinal)
        C_f = self.params.cornering_stiffness_front
        C_r = self.params.cornering_stiffness_rear

        K_us_deg = (W_f / C_f - W_r / C_r) * 180 / math.pi
        ay_g = lateral_accel / 9.81

        K_us = K_us_deg / (ay_g + 0.001)

        return K_us * friction_coeff

    def _weight_distribution(self, cg_longitudinal: float) -> Tuple[float, float]:
        L = self.params.wheelbase
        m_total = self.params.mass + self.cargo.mass
        W = m_total * 9.81
        W_f = W * (0.5 - cg_longitudinal / L)
        W_r = W * (0.5 + cg_longitudinal / L)
        return max(W_f, 1.0), max(W_r, 1.0)

    def calculate_stability_margin(self, speed: float, pole_angle_deg: float,
                                   friction_coeff: float = 0.7) -> dict:
        critical_speed = self.rollover_risk.calculate_critical_speed(
            friction_coeff, pole_angle_deg
        )

        speed_margin = critical_speed - speed
        speed_margin_pct = (speed_margin / critical_speed * 100) if critical_speed > 0 else 0

        _, risk_level, details = self.rollover_risk.calculate_rollover_risk(
            speed, pole_angle_deg, friction_coeff, 0.0
        )

        return {
            "critical_speed": critical_speed,
            "current_speed": speed,
            "speed_margin": speed_margin,
            "speed_margin_percent": speed_margin_pct,
            "risk_level": risk_level,
            "inner_wheel_load": details.get("inner_wheel_load_ratio", 0.5),
            "ssf": details.get("ssf", 0)
        }


if __name__ == "__main__":
    cargo = CargoConfig(
        mass=200,
        offset_lateral=0.05,
        offset_longitudinal=0.2,
        offset_height=0.3
    )
    analyzer = StabilityAnalyzer(cargo=cargo)
    result = analyzer.analyze(
        speed=8.0,
        pole_angle_deg=15.0,
        roll_angle_deg=12.0,
        slip_rate=0.15,
        friction_coeff=0.6,
        dt=60.0
    )
    print(f"有效重心高度: {result.effective_cg_height:.3f} m")
    print(f"有效重心横向偏移: {result.effective_cg_lateral:.3f} m")
    print(f"货物横向移动: {result.cargo_shift_lateral:.4f} m")
    print(f"横摆惯量: {result.effective_yaw_inertia:.1f} kg·m²")
    print(f"横摆角速度: {result.yaw_rate:.2f}°/s")
    print(f"侧翻风险: {result.rollover_risk:.1f}%")
    print(f"稳定性指数: {result.stability_index:.2f}")
