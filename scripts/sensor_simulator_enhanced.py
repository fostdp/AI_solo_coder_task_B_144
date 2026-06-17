import sys
import os
import time
import json
import random
import math
import requests
import argparse
from typing import Dict, List, Optional
from dataclasses import dataclass, field

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


@dataclass
class RoadCondition:
    name: str
    friction_min: float
    friction_max: float
    slip_factor: float = 1.0
    description: str = ""


ROAD_CONDITIONS = {
    "dry_asphalt": RoadCondition(
        name="干燥柏油路",
        friction_min=0.7,
        friction_max=0.9,
        slip_factor=0.8,
        description="理想路况，抓地力强"
    ),
    "wet_asphalt": RoadCondition(
        name="湿滑柏油路",
        friction_min=0.4,
        friction_max=0.6,
        slip_factor=1.5,
        description="雨后路面，制动距离增加"
    ),
    "gravel": RoadCondition(
        name="碎石路",
        friction_min=0.5,
        friction_max=0.7,
        slip_factor=1.8,
        description="不平整路面，颠簸明显"
    ),
    "muddy": RoadCondition(
        name="泥泞路",
        friction_min=0.2,
        friction_max=0.4,
        slip_factor=2.5,
        description="泥泞土路，极易打滑"
    ),
    "icy": RoadCondition(
        name="冰雪路",
        friction_min=0.1,
        friction_max=0.25,
        slip_factor=3.0,
        description="冰雪覆盖，极度危险"
    ),
    "dirt_track": RoadCondition(
        name="古代驿道",
        friction_min=0.35,
        friction_max=0.55,
        slip_factor=1.6,
        description="秦汉时期典型土路"
    )
}


@dataclass
class ScenarioConfig:
    name: str
    turning_radius_min: float
    turning_radius_max: float
    speed_min: float
    speed_max: float
    road_condition: str
    pole_angle_min: float
    pole_angle_max: float
    description: str = ""


SCENARIOS = {
    "straight_cruise": ScenarioConfig(
        name="直行巡航",
        turning_radius_min=100.0,
        turning_radius_max=500.0,
        speed_min=3.0,
        speed_max=7.0,
        road_condition="dry_asphalt",
        pole_angle_min=-2.0,
        pole_angle_max=2.0,
        description="平稳直行，辕杆小幅度修正"
    ),
    "gentle_turns": ScenarioConfig(
        name="平缓弯道",
        turning_radius_min=20.0,
        turning_radius_max=50.0,
        speed_min=4.0,
        speed_max=6.0,
        road_condition="dry_asphalt",
        pole_angle_min=-15.0,
        pole_angle_max=15.0,
        description="大半径弯道，侧倾角小"
    ),
    "sharp_turns": ScenarioConfig(
        name="急弯行驶",
        turning_radius_min=5.0,
        turning_radius_max=15.0,
        speed_min=2.0,
        speed_max=4.0,
        road_condition="dirt_track",
        pole_angle_min=-35.0,
        pole_angle_max=35.0,
        description="小半径弯道，侧倾风险高"
    ),
    "wet_road": ScenarioConfig(
        name="湿滑路面",
        turning_radius_min=15.0,
        turning_radius_max=40.0,
        speed_min=2.5,
        speed_max=4.5,
        road_condition="wet_asphalt",
        pole_angle_min=-20.0,
        pole_angle_max=20.0,
        description="湿滑路面，滑移率升高"
    ),
    "off_road": ScenarioConfig(
        name="越野路况",
        turning_radius_min=10.0,
        turning_radius_max=30.0,
        speed_min=1.5,
        speed_max=3.5,
        road_condition="muddy",
        pole_angle_min=-30.0,
        pole_angle_max=30.0,
        description="泥泞越野，极易侧翻"
    ),
    "highway": ScenarioConfig(
        name="高速行驶",
        turning_radius_min=50.0,
        turning_radius_max=100.0,
        speed_min=8.0,
        speed_max=12.0,
        road_condition="dry_asphalt",
        pole_angle_min=-10.0,
        pole_angle_max=10.0,
        description="高速行驶，横摆风险高"
    ),
    "ancient_road": ScenarioConfig(
        name="古代驿道",
        turning_radius_min=8.0,
        turning_radius_max=25.0,
        speed_min=2.0,
        speed_max=4.0,
        road_condition="dirt_track",
        pole_angle_min=-25.0,
        pole_angle_max=25.0,
        description="复原秦汉时期典型路况"
    )
}


class ChariotSensorSimulator:
    def __init__(self, vehicle_id: str, scenario: str = "ancient_road",
                 base_speed: Optional[float] = None,
                 road_override: Optional[str] = None,
                 turning_radius_override: Optional[float] = None):
        self.vehicle_id = vehicle_id
        self.scenario_name = scenario
        self.scenario = SCENARIOS.get(scenario, SCENARIOS["ancient_road"])

        if road_override and road_override in ROAD_CONDITIONS:
            self.road_condition = ROAD_CONDITIONS[road_override]
        else:
            self.road_condition = ROAD_CONDITIONS[self.scenario.road_condition]

        self.turning_radius_override = turning_radius_override

        self.base_speed = base_speed if base_speed else random.uniform(
            self.scenario.speed_min, self.scenario.speed_max
        )

        self.time_elapsed = 0.0
        self.pole_angle = 0.0
        self.slip_rate = 0.1
        self.roll_angle = 0.0
        self.friction_coeff = 0.7
        self.turn_state = 0
        self.turn_target = 0
        self.current_turning_radius = 0.0
        self.lateral_accel = 0.0
        self.yaw_rate = 0.0

        self.road_condition_change_counter = 0
        self.road_change_interval = random.randint(10, 20)

        wheelbase = 2.5
        self.L = wheelbase

    def set_scenario(self, scenario: str):
        if scenario in SCENARIOS:
            self.scenario_name = scenario
            self.scenario = SCENARIOS[scenario]
            self.road_condition = ROAD_CONDITIONS[self.scenario.road_condition]
            print(f"[{self.vehicle_id}] 切换场景: {self.scenario.name}")

    def set_road_condition(self, condition: str):
        if condition in ROAD_CONDITIONS:
            self.road_condition = ROAD_CONDITIONS[condition]
            print(f"[{self.vehicle_id}] 切换路面: {self.road_condition.name}")

    def calculate_pole_from_radius(self, turning_radius: float, direction: int) -> float:
        if abs(turning_radius) < 0.1:
            return direction * 40.0

        R_inner = turning_radius - 0.9
        if R_inner < 0.1:
            R_inner = 0.1

        pole_angle = math.degrees(math.atan(self.L / turning_radius))
        return direction * min(max(pole_angle, self.scenario.pole_angle_min), self.scenario.pole_angle_max)

    def update(self, dt: float = 60.0):
        self.time_elapsed += dt
        self.road_condition_change_counter += 1

        if self.road_condition_change_counter >= self.road_change_interval:
            self.road_condition_change_counter = 0
            self.road_change_interval = random.randint(10, 20)
            if random.random() < 0.3:
                conditions = list(ROAD_CONDITIONS.keys())
                new_condition = random.choice(conditions)
                self.road_condition = ROAD_CONDITIONS[new_condition]
                print(f"[{self.vehicle_id}] 路面变化: {self.road_condition.name}")

        if self.turning_radius_override:
            desired_radius = self.turning_radius_override
            direction = 1 if self.turning_radius_override > 0 else -1
            self.turn_target = self.calculate_pole_from_radius(abs(desired_radius), direction)
            self.turn_state = 1
        else:
            if self.turn_state == 0 and random.random() < 0.2:
                direction = random.choice([-1, 1])
                desired_radius = random.uniform(
                    self.scenario.turning_radius_min,
                    self.scenario.turning_radius_max
                )
                self.turn_target = self.calculate_pole_from_radius(desired_radius, direction)
                self.turn_state = 1
                self.current_turning_radius = desired_radius * direction
                print(f"[{self.vehicle_id}] 进入弯道: R={abs(desired_radius):.1f}m, "
                      f"目标辕杆角={self.turn_target:.1f}°")

        if self.turn_state >= 1:
            angle_diff = self.turn_target - self.pole_angle
            self.pole_angle += angle_diff * 0.25

            if abs(angle_diff) < 0.5:
                self.turn_state = 2

            if self.turn_state == 2 and random.random() < 0.25:
                self.turn_target = 0
                self.turn_state = 1
                self.current_turning_radius = 0
                print(f"[{self.vehicle_id}] 驶出弯道，回正方向")
        else:
            self.pole_angle += random.uniform(-1.0, 1.0)
            self.pole_angle = max(-3, min(3, self.pole_angle))

        self.pole_angle = max(self.scenario.pole_angle_min, min(self.scenario.pole_angle_max, self.pole_angle))

        turn_factor = abs(self.pole_angle) / 40.0
        speed = self.base_speed * (1 - turn_factor * 0.3)

        if abs(self.pole_angle) > 1.0:
            R = self.L / math.tan(math.radians(abs(self.pole_angle)))
            self.lateral_accel = (speed ** 2) / R * 9.81
            self.yaw_rate = math.degrees(speed / R)
        else:
            self.lateral_accel = 0
            self.yaw_rate = 0

        base_slip = 0.05 + turn_factor * 0.2
        self.slip_rate = base_slip * self.road_condition.slip_factor + random.uniform(-0.03, 0.03)

        if random.random() < 0.04:
            self.slip_rate = random.uniform(0.7, 0.95)
        elif random.random() < 0.02:
            self.slip_rate = random.uniform(0.01, 0.04)

        self.slip_rate = max(0.0, min(1.0, self.slip_rate))

        self.roll_angle = self.lateral_accel * 1.5 + random.uniform(-1.5, 1.5)

        if random.random() < 0.06:
            self.roll_angle = random.uniform(22, 32)

        self.roll_angle = max(-35, min(35, self.roll_angle))

        self.friction_coeff = random.uniform(
            self.road_condition.friction_min,
            self.road_condition.friction_max
        )

    def get_data(self) -> dict:
        return {
            "vehicle_id": self.vehicle_id,
            "pole_angle": round(self.pole_angle, 3),
            "slip_rate": round(self.slip_rate, 4),
            "roll_angle": round(self.roll_angle, 3),
            "friction_coeff": round(self.friction_coeff, 4),
            "turning_radius": round(self.current_turning_radius, 2),
            "lateral_accel": round(self.lateral_accel, 3),
            "yaw_rate": round(self.yaw_rate, 3),
            "speed": round(self.base_speed, 2),
            "scenario": self.scenario_name,
            "road_condition": self.road_condition.name,
            "timestamp": int(time.time())
        }

    def send_data(self, api_url: str) -> bool:
        data = self.get_data()
        try:
            response = requests.post(f"{api_url}/api/sensor/data", json=data, timeout=5)
            if response.status_code == 200:
                print(f"[{self.vehicle_id}] 数据已发送 - 场景:{self.scenario.name[:4]} "
                      f"路面:{self.road_condition.name[:4]} "
                      f"转角:{data['pole_angle']:.1f}° "
                      f"R={abs(data['turning_radius']):.1f}m "
                      f"侧倾:{data['roll_angle']:.1f}° "
                      f"摩擦:{data['friction_coeff']:.2f}")
                return True
            else:
                print(f"[{self.vehicle_id}] 发送失败: {response.status_code} - {response.text}")
                return False
        except Exception as e:
            print(f"[{self.vehicle_id}] 连接错误: {e}")
            return False


def list_scenarios():
    print("\n=== 可用场景 ===")
    for key, sc in SCENARIOS.items():
        print(f"  {key:20s} - {sc.name}: {sc.description}")
        print(f"              转弯半径: {sc.turning_radius_min}-{sc.turning_radius_max}m, "
              f"速度: {sc.speed_min}-{sc.speed_max}m/s, "
              f"路面: {ROAD_CONDITIONS[sc.road_condition].name}")
    print()


def list_road_conditions():
    print("\n=== 可用路面条件 ===")
    for key, rc in ROAD_CONDITIONS.items():
        print(f"  {key:15s} - {rc.name}: {rc.description}")
        print(f"              摩擦系数: {rc.friction_min}-{rc.friction_max}, "
              f"滑移因子: {rc.slip_factor}")
    print()


def main():
    parser = argparse.ArgumentParser(description="秦汉双辕车传感器模拟器 v2.0")
    parser.add_argument("--api-host", default="api_gateway", help="API网关主机")
    parser.add_argument("--api-port", type=int, default=8000, help="API网关端口")
    parser.add_argument("--interval", type=int, default=60, help="上报间隔(秒)")
    parser.add_argument("--vehicles", type=int, default=3, help="模拟车辆数量")
    parser.add_argument("--scenario", default="ancient_road",
                        help=f"运行场景: {', '.join(SCENARIOS.keys())}")
    parser.add_argument("--road", default=None,
                        help=f"路面条件覆盖: {', '.join(ROAD_CONDITIONS.keys())}")
    parser.add_argument("--turning-radius", type=float, default=None,
                        help="固定转弯半径(米), 正值右转, 负值左转")
    parser.add_argument("--speed", type=float, default=None, help="固定车速(m/s)")
    parser.add_argument("--list-scenarios", action="store_true", help="列出所有场景")
    parser.add_argument("--list-roads", action="store_true", help="列出所有路面条件")

    args = parser.parse_args()

    if args.list_scenarios:
        list_scenarios()
        return

    if args.list_roads:
        list_road_conditions()
        return

    API_URL = f"http://{args.api_host}:{args.api_port}"

    vehicle_ids = [f"chariot-{i:03d}" for i in range(1, args.vehicles + 1)]

    simulators = []
    for i, vid in enumerate(vehicle_ids):
        scenario = args.scenario
        road = args.road
        if not road and i > 0:
            scenarios = list(SCENARIOS.keys())
            scenario = random.choice(scenarios)
            roads = list(ROAD_CONDITIONS.keys())
            road = random.choice(roads)

        sim = ChariotSensorSimulator(
            vid,
            scenario=scenario,
            base_speed=args.speed,
            road_override=road,
            turning_radius_override=args.turning_radius
        )
        simulators.append(sim)

    print("=" * 70)
    print("秦汉双辕车传感器模拟器 v2.0")
    print("=" * 70)
    print(f"目标API: {API_URL}")
    print(f"车辆数量: {len(vehicle_ids)}")
    print(f"上报间隔: {args.interval}秒")
    print(f"默认场景: {SCENARIOS.get(args.scenario, {}).get('name', args.scenario)}")
    if args.road:
        print(f"路面覆盖: {ROAD_CONDITIONS.get(args.road, {}).get('name', args.road)}")
    if args.turning_radius:
        print(f"固定转弯半径: {args.turning_radius}m")
    if args.speed:
        print(f"固定车速: {args.speed}m/s")
    print("=" * 70)

    print("\n车辆配置:")
    for sim in simulators:
        print(f"  {sim.vehicle_id}: 场景={sim.scenario.name}, 路面={sim.road_condition.name}")

    print(f"\n首次发送数据...")
    for sim in simulators:
        sim.send_data(API_URL)

    print(f"\n进入循环模式，每 {args.interval} 秒上报一次...")
    print("按 Ctrl+C 停止\n")

    try:
        while True:
            time.sleep(args.interval)
            for sim in simulators:
                sim.update(args.interval)
                sim.send_data(API_URL)
    except KeyboardInterrupt:
        print("\n\n模拟器已停止。")


if __name__ == "__main__":
    main()
