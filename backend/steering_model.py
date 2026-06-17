import math
import numpy as np
from dataclasses import dataclass
from typing import Tuple, Optional


@dataclass
class ChariotParams:
    wheelbase: float = 2.5
    track_width: float = 1.8
    wheel_radius: float = 0.35
    pole_length: float = 1.8
    kingpin_offset: float = 0.1
    steering_arm_length: float = 0.25
    tie_rod_length: float = -1.0
    ackermann_angle_deg: float = 12.0

    def __post_init__(self):
        if self.tie_rod_length < 0:
            T = self.track_width - 2 * self.kingpin_offset
            L = self.steering_arm_length
            ack = math.radians(self.ackermann_angle_deg)
            self.tie_rod_length = math.sqrt(
                (T - 2 * L * math.sin(ack)) ** 2
            )


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


@dataclass
class FourBarSolution:
    inner_angle: float
    outer_angle: float
    transmission_angle_inner: float
    transmission_angle_outer: float
    valid: bool
    interference: bool
    dead_point: bool


class FourBarLinkageSolver:
    def __init__(self, params: ChariotParams = None):
        self.params = params or ChariotParams()
        self.MIN_TRANSMISSION_ANGLE = math.radians(30)
        self.MAX_TRANSMISSION_ANGLE = math.radians(150)

    def _freudenstein_solve(self, input_angle: float, r1: float, r2: float,
                            r3: float, r4: float) -> Optional[float]:
        K1 = r1 / r2
        K2 = r1 / r4
        K3 = (r2 ** 2 - r3 ** 2 + r4 ** 2 + r1 ** 2) / (2 * r2 * r4)
        K4 = r1 / r3
        K5 = (r4 ** 2 - r1 ** 2 - r2 ** 2 - r3 ** 2) / (2 * r2 * r3)

        cos_theta2 = math.cos(input_angle)
        sin_theta2 = math.sin(input_angle)

        A = cos_theta2 - K1 - K2 * cos_theta2 + K3
        B = -2 * sin_theta2
        C = K1 - (K2 + 1) * cos_theta2 + K3

        discriminant = B ** 2 - 4 * A * C
        if discriminant < 0:
            return None

        sqrt_d = math.sqrt(discriminant)
        tan_half_1 = (-B + sqrt_d) / (2 * A) if abs(A) > 1e-10 else None
        tan_half_2 = (-B - sqrt_d) / (2 * A) if abs(A) > 1e-10 else None

        solutions = []
        for tan_half in [tan_half_1, tan_half_2]:
            if tan_half is not None and not math.isnan(tan_half):
                try:
                    theta = 2 * math.atan(tan_half)
                    solutions.append(theta)
                except:
                    pass

        if not solutions:
            return None

        valid_solutions = []
        for sol in solutions:
            cos_theta4 = math.cos(sol)
            sin_theta4 = math.sin(sol)
            D = cos_theta4 - K1 + K4 * cos_theta4 + K5
            E = -2 * sin_theta4
            F = K1 + (K4 - 1) * cos_theta4 + K5
            disc2 = E ** 2 - 4 * D * F
            if disc2 >= -1e-6:
                valid_solutions.append(sol)

        if not valid_solutions:
            return solutions[0]

        best_sol = valid_solutions[0]
        for sol in valid_solutions:
            if abs(sol) < abs(best_sol):
                best_sol = sol
        return best_sol

    def _freudenstein_solve_with_ref(self, input_angle: float, ref_output: float,
                                      r1: float, r2: float, r3: float, r4: float) -> Optional[float]:
        K1 = r1 / r2
        K2 = r1 / r4
        K3 = (r2 ** 2 - r3 ** 2 + r4 ** 2 + r1 ** 2) / (2 * r2 * r4)

        cos_theta2 = math.cos(input_angle)
        sin_theta2 = math.sin(input_angle)

        A = cos_theta2 - K1 - K2 * cos_theta2 + K3
        B = -2 * sin_theta2
        C = K1 - (K2 + 1) * cos_theta2 + K3

        discriminant = B ** 2 - 4 * A * C
        if discriminant < -1e-8:
            return None
        discriminant = max(discriminant, 0.0)

        sqrt_d = math.sqrt(discriminant)
        candidates = []
        for sign in [+1, -1]:
            if abs(A) < 1e-10:
                if abs(B) < 1e-10:
                    continue
                tan_half = -C / B
            else:
                tan_half = (-B + sign * sqrt_d) / (2 * A)
            try:
                theta = 2 * math.atan(tan_half)
                candidates.append(theta)
                candidates.append(theta + 2 * math.pi)
                candidates.append(theta - 2 * math.pi)
            except:
                pass

        if not candidates:
            return None

        best = min(candidates, key=lambda t: abs(t - ref_output))
        return best

    def _compute_transmission_angle(self, theta2: float, theta4: float,
                                     r1: float, r2: float, r3: float, r4: float) -> float:
        x3 = r2 * math.cos(theta2) + r3 * 0
        y3 = r2 * math.sin(theta2)
        x_coupler_end = r1 + r4 * math.cos(theta4)
        y_coupler_end = r4 * math.sin(theta4)

        dx = x_coupler_end - x3
        dy = y_coupler_end - y3
        coupler_angle = math.atan2(dy, dx)

        input_link_angle = theta2

        mu = abs(coupler_angle - input_link_angle)
        while mu > math.pi:
            mu -= 2 * math.pi
        mu = abs(mu)
        if mu > math.pi:
            mu = 2 * math.pi - mu
        return mu

    def _check_interference(self, theta_left: float, theta_right: float,
                             arm_len: float, kingpin_distance: float) -> bool:
        left_end_x = -kingpin_distance / 2 - arm_len * math.sin(theta_left)
        left_end_y = arm_len * math.cos(theta_left)

        right_end_x = kingpin_distance / 2 + arm_len * math.sin(theta_right)
        right_end_y = arm_len * math.cos(theta_right)

        min_distance = 0.05
        actual_distance = math.sqrt(
            (right_end_x - left_end_x) ** 2 + (right_end_y - left_end_y) ** 2
        )

        if actual_distance < min_distance * 2:
            return True

        tie_rod_mid_y = (left_end_y + right_end_y) / 2
        if tie_rod_mid_y < -0.05:
            return True

        return False

    def solve(self, inner_wheel_target: float, direction: int = 1) -> FourBarSolution:
        T = self.params.track_width
        L_arm = self.params.steering_arm_length
        L_tie = self.params.tie_rod_length
        ack_angle = math.radians(self.params.ackermann_angle_deg)

        kingpin_distance = T - 2 * self.params.kingpin_offset
        r1 = kingpin_distance
        r2 = L_arm
        r3 = L_tie
        r4 = L_arm

        theta2_initial = math.pi / 2 - ack_angle
        theta4_initial = math.pi / 2 + ack_angle
        inner_target_abs = abs(inner_wheel_target)

        if direction > 0:
            theta2 = theta2_initial + inner_target_abs
        else:
            theta2 = theta2_initial - inner_target_abs

        theta4 = self._freudenstein_solve_with_ref(theta2, theta4_initial, r1, r2, r3, r4)

        if theta4 is None:
            return FourBarSolution(0, 0, 0, 0, False, True, True)

        if direction > 0:
            outer_angle = theta4 - theta4_initial
        else:
            outer_angle = theta4_initial - theta4

        mu_inner = self._compute_transmission_angle(theta2, theta4, r1, r2, r3, r4)
        mu_outer = self._compute_transmission_angle(theta4, theta2, r1, r4, r3, r2)

        valid = True
        dead_point = False
        interference = False

        if (mu_inner < self.MIN_TRANSMISSION_ANGLE or
            mu_inner > self.MAX_TRANSMISSION_ANGLE or
            mu_outer < self.MIN_TRANSMISSION_ANGLE or
            mu_outer > self.MAX_TRANSMISSION_ANGLE):
            dead_point = True
            valid = False

        left_arm_x = r2 * math.cos(theta2)
        left_arm_y = r2 * math.sin(theta2)
        right_arm_x = r1 + r4 * math.cos(theta4)
        right_arm_y = r4 * math.sin(theta4)

        min_distance = 0.05
        dist_arms = math.sqrt((right_arm_x - left_arm_x) ** 2 + (right_arm_y - left_arm_y) ** 2)
        if dist_arms < min_distance * 2:
            interference = True
            valid = False

        if (left_arm_y < -0.05) or (right_arm_y < -0.05):
            interference = True
            valid = False

        tie_rod_mid_y = (left_arm_y + right_arm_y) / 2
        if tie_rod_mid_y < 0.05:
            interference = True
            valid = False

        actual_inner = inner_wheel_target * direction
        actual_outer = outer_angle * direction

        return FourBarSolution(
            inner_angle=actual_inner,
            outer_angle=actual_outer,
            transmission_angle_inner=math.degrees(mu_inner),
            transmission_angle_outer=math.degrees(mu_outer),
            valid=valid,
            interference=interference,
            dead_point=dead_point
        )

    def find_max_steering_angle(self) -> Tuple[float, float]:
        max_inner_rad = 0.0
        max_pole_rad = 0.0
        for angle_deg in range(1, 50):
            angle_rad = math.radians(angle_deg)
            sol_pos = self.solve(angle_rad, 1)
            sol_neg = self.solve(angle_rad, -1)
            if sol_pos.valid and sol_neg.valid:
                max_inner_rad = angle_rad
                L = self.params.wheelbase
                T = self.params.track_width
                R = L / math.tan(angle_rad) + T / 2
                max_pole_rad = math.atan(L / R)
            else:
                break
        return max_inner_rad, max_pole_rad


class AckermannSteeringModel:
    def __init__(self, params: ChariotParams = None):
        self.params = params or ChariotParams()
        self.four_bar = FourBarLinkageSolver(params)

    def _pole_to_inner_wheel(self, pole_angle_rad: float) -> Tuple[float, int]:
        L = self.params.wheelbase
        T = self.params.track_width

        direction = 1 if pole_angle_rad >= 0 else -1
        pole_abs = abs(pole_angle_rad)

        if pole_abs < 0.0001:
            return 0.0, direction

        R_ideal = L / math.tan(pole_abs)

        R_inner = R_ideal - T / 2
        if R_inner < 0.1:
            R_inner = 0.1

        inner_angle = math.atan(L / R_inner)

        max_inner_rad = math.radians(45)
        inner_angle = min(inner_angle, max_inner_rad)

        return inner_angle, direction

    def calculate_ackermann_geometry(self, pole_angle_deg: float) -> SteeringResult:
        pole_angle = math.radians(pole_angle_deg)
        L = self.params.wheelbase
        T = self.params.track_width
        d = self.params.kingpin_offset

        if abs(pole_angle) < 0.001:
            return SteeringResult(
                inner_wheel_angle=0.0,
                outer_wheel_angle=0.0,
                turning_radius=float('inf'),
                wheel_speed_diff=0.0,
                ackermann_error=0.0,
                pole_effective_angle=pole_angle_deg,
                transmission_angle_inner=90.0,
                transmission_angle_outer=90.0,
                linkage_interference=False,
                dead_point_risk=False
            )

        inner_target, direction = self._pole_to_inner_wheel(pole_angle)

        four_bar_sol = self.four_bar.solve(inner_target, direction)
        limited = False

        if not four_bar_sol.valid:
            max_inner_rad, max_pole_rad = self.four_bar.find_max_steering_angle()
            safe_pole = max_pole_rad * 0.95 * direction
            safe_inner = max_inner_rad * 0.95
            four_bar_sol = self.four_bar.solve(safe_inner, direction)
            limited = True
            if not four_bar_sol.valid:
                inner_angle = inner_target
                outer_angle = math.atan(L / (L / math.tan(abs(inner_angle)) + T))
                if direction < 0:
                    inner_angle = -inner_angle
                    outer_angle = -outer_angle
                if direction > 0:
                    inner_angle, outer_angle = outer_angle, inner_angle
                R_actual = L / math.tan(abs(inner_angle)) + T / 2 - d * math.sin(abs(inner_angle))
                wheel_speed_diff = T / (2 * R_actual) if R_actual > 0 else 0
                ack_err = abs(1 / math.tan(outer_angle) - 1 / math.tan(inner_angle) + T / L) / (T / L) if abs(inner_angle) > 0.001 else 0
                return SteeringResult(
                    inner_wheel_angle=math.degrees(inner_angle),
                    outer_wheel_angle=math.degrees(outer_angle),
                    turning_radius=R_actual,
                    wheel_speed_diff=wheel_speed_diff,
                    ackermann_error=ack_err,
                    pole_effective_angle=math.degrees(safe_pole) if safe_pole != 0 else pole_angle_deg,
                    transmission_angle_inner=45.0,
                    transmission_angle_outer=45.0,
                    linkage_interference=four_bar_sol.interference,
                    dead_point_risk=True
                )

        inner_angle_raw = four_bar_sol.inner_angle
        outer_angle_raw = four_bar_sol.outer_angle

        if direction > 0:
            inner_angle = outer_angle_raw
            outer_angle = inner_angle_raw
            trans_inner = four_bar_sol.transmission_angle_outer
            trans_outer = four_bar_sol.transmission_angle_inner
        else:
            inner_angle = inner_angle_raw
            outer_angle = outer_angle_raw
            trans_inner = four_bar_sol.transmission_angle_inner
            trans_outer = four_bar_sol.transmission_angle_outer

        inner_abs = abs(inner_angle)
        if inner_abs < 0.001:
            R_actual = float('inf')
        else:
            R_actual = L / math.tan(inner_abs) + T / 2 - d * math.sin(inner_abs)

        wheel_speed_diff = T / (2 * R_actual) if R_actual != float('inf') and R_actual > 0 else 0

        if abs(inner_angle) > 0.001 and abs(outer_angle) > 0.001:
            ack_err = abs(1 / math.tan(outer_angle) - 1 / math.tan(inner_angle) + T / L) / (T / L)
        else:
            ack_err = 0.0

        return SteeringResult(
            inner_wheel_angle=math.degrees(inner_angle),
            outer_wheel_angle=math.degrees(outer_angle),
            turning_radius=R_actual,
            wheel_speed_diff=wheel_speed_diff,
            ackermann_error=ack_err,
            pole_effective_angle=pole_angle_deg if not limited else math.degrees(math.atan(L / (R_actual - T / 2 + d * math.sin(abs(inner_angle))))),
            transmission_angle_inner=trans_inner,
            transmission_angle_outer=trans_outer,
            linkage_interference=four_bar_sol.interference,
            dead_point_risk=four_bar_sol.dead_point or limited
        )


class MultiBodyDynamicsSteering:
    def __init__(self, params: ChariotParams = None):
        self.params = params or ChariotParams()
        self.ackermann = AckermannSteeringModel(params)

    def _linkage_kinematics(self, pole_angle: float) -> Tuple[float, float]:
        ack_result = self.ackermann.calculate_ackermann_geometry(pole_angle)
        inner_angle = math.radians(ack_result.inner_wheel_angle)
        outer_angle = math.radians(ack_result.outer_wheel_angle)
        return inner_angle, outer_angle

    def calculate_steering(self, pole_angle: float, vehicle_speed: float = 5.0,
                           friction_coeff: float = 0.7) -> SteeringResult:
        ack_result = self.ackermann.calculate_ackermann_geometry(pole_angle)

        inner_angle = math.radians(ack_result.inner_wheel_angle)
        outer_angle = math.radians(ack_result.outer_wheel_angle)

        L = self.params.wheelbase
        T = self.params.track_width

        avg_angle = (abs(inner_angle) + abs(outer_angle)) / 2
        if avg_angle > 0.001:
            actual_radius = L / math.tan(avg_angle) + T / 4
        else:
            actual_radius = float('inf')

        slip_factor = 1.0 - (friction_coeff - 0.3) * 0.2
        if actual_radius != float('inf'):
            actual_radius *= slip_factor

        if actual_radius != float('inf') and abs(actual_radius) > 0.001:
            speed_diff = (T / 2) / abs(actual_radius)
        else:
            speed_diff = 0

        if ack_result.dead_point_risk or ack_result.linkage_interference:
            speed_diff *= 0.7

        return SteeringResult(
            inner_wheel_angle=ack_result.inner_wheel_angle,
            outer_wheel_angle=ack_result.outer_wheel_angle,
            turning_radius=actual_radius,
            wheel_speed_diff=speed_diff,
            ackermann_error=ack_result.ackermann_error,
            pole_effective_angle=ack_result.pole_effective_angle,
            transmission_angle_inner=ack_result.transmission_angle_inner,
            transmission_angle_outer=ack_result.transmission_angle_outer,
            linkage_interference=ack_result.linkage_interference,
            dead_point_risk=ack_result.dead_point_risk
        )

    def get_wheel_trajectory(self, pole_angle: float, speed: float = 5.0,
                             duration: float = 10.0, dt: float = 0.1) -> dict:
        result = self.calculate_steering(pole_angle, speed)

        num_steps = int(duration / dt)
        x_inner = np.zeros(num_steps)
        y_inner = np.zeros(num_steps)
        x_outer = np.zeros(num_steps)
        y_outer = np.zeros(num_steps)
        x_center = np.zeros(num_steps)
        y_center = np.zeros(num_steps)

        R = result.turning_radius
        T = self.params.track_width

        if R == float('inf') or abs(R) > 1000:
            for i in range(num_steps):
                s = speed * i * dt
                x_inner[i] = s
                y_inner[i] = -T / 2
                x_outer[i] = s
                y_outer[i] = T / 2
                x_center[i] = s
                y_center[i] = 0
        else:
            direction = 1 if pole_angle > 0 else -1
            R_abs = abs(R)
            angular_vel = speed / R_abs

            for i in range(num_steps):
                theta = angular_vel * i * dt
                x_center[i] = R_abs * math.sin(theta) * direction
                y_center[i] = R_abs * (1 - math.cos(theta))

                R_inner = R_abs - T / 2
                R_outer = R_abs + T / 2

                x_inner[i] = R_inner * math.sin(theta) * direction
                y_inner[i] = R_inner * (1 - math.cos(theta))
                x_outer[i] = R_outer * math.sin(theta) * direction
                y_outer[i] = R_outer * (1 - math.cos(theta))

        return {
            "inner_wheel": {"x": x_inner.tolist(), "y": y_inner.tolist()},
            "outer_wheel": {"x": x_outer.tolist(), "y": y_outer.tolist()},
            "center": {"x": x_center.tolist(), "y": y_center.tolist()},
            "turning_radius": R,
            "duration": duration
        }

    def get_linkage_positions(self, pole_angle: float) -> dict:
        L = self.params.wheelbase
        T = self.params.track_width
        arm_len = self.params.steering_arm_length

        inner_angle, outer_angle = self._linkage_kinematics(pole_angle)

        ack_angle = math.radians(self.params.ackermann_angle_deg)

        left_knuckle_x = -T / 2
        left_knuckle_y = 0
        right_knuckle_x = T / 2
        right_knuckle_y = 0

        if pole_angle > 0:
            left_arm_abs = outer_angle
            right_arm_abs = inner_angle
        else:
            left_arm_abs = -inner_angle
            right_arm_abs = -outer_angle

        left_total_angle = math.pi / 2 - ack_angle + left_arm_abs
        right_total_angle = math.pi / 2 + ack_angle + right_arm_abs

        left_arm_end_x = left_knuckle_x + arm_len * math.cos(left_total_angle)
        left_arm_end_y = left_knuckle_y + arm_len * math.sin(left_total_angle)

        right_arm_end_x = right_knuckle_x + arm_len * math.cos(right_total_angle)
        right_arm_end_y = right_knuckle_y + arm_len * math.sin(right_total_angle)

        pole_base_x = 0
        pole_base_y = L / 2

        pole_angle_rad = math.radians(pole_angle)
        pole_tip_x = pole_base_x + self.params.pole_length * math.sin(pole_angle_rad)
        pole_tip_y = pole_base_y + self.params.pole_length * math.cos(pole_angle_rad)

        return {
            "left_knuckle": {"x": left_knuckle_x, "y": left_knuckle_y},
            "right_knuckle": {"x": right_knuckle_x, "y": right_knuckle_y},
            "left_arm_end": {"x": left_arm_end_x, "y": left_arm_end_y},
            "right_arm_end": {"x": right_arm_end_x, "y": right_arm_end_y},
            "tie_rod_left": {"x": left_arm_end_x, "y": left_arm_end_y},
            "tie_rod_right": {"x": right_arm_end_x, "y": right_arm_end_y},
            "pole_base": {"x": pole_base_x, "y": pole_base_y},
            "pole_tip": {"x": pole_tip_x, "y": pole_tip_y},
            "left_wheel_angle": math.degrees(outer_angle),
            "right_wheel_angle": math.degrees(inner_angle)
        }
