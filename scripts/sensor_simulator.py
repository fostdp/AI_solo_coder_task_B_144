import sys
import os
import time
import json
import random
import math
import requests

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from config.settings import FASTAPI_HOST, FASTAPI_PORT


API_URL = f"http://{FASTAPI_HOST if FASTAPI_HOST != '0.0.0.0' else 'localhost'}:{FASTAPI_PORT}"


class ChariotSensorSimulator:
    def __init__(self, vehicle_id: str, base_speed: float = 5.0):
        self.vehicle_id = vehicle_id
        self.base_speed = base_speed
        self.time_elapsed = 0.0
        self.pole_angle = 0.0
        self.slip_rate = 0.1
        self.roll_angle = 0.0
        self.friction_coeff = 0.7
        self.turn_state = 0
        self.turn_target = 0

    def update(self, dt: float = 60.0):
        self.time_elapsed += dt

        if random.random() < 0.15:
            self.turn_target = random.uniform(-35, 35)
            self.turn_state = 1

        if self.turn_state == 1:
            angle_diff = self.turn_target - self.pole_angle
            self.pole_angle += angle_diff * 0.3

            if abs(angle_diff) < 1.0:
                self.turn_state = 2
        elif self.turn_state == 2:
            if random.random() < 0.3:
                self.turn_target = 0
                self.turn_state = 1
        else:
            self.pole_angle += random.uniform(-2, 2)
            self.pole_angle = max(-5, min(5, self.pole_angle))

        self.pole_angle = max(-40, min(40, self.pole_angle))

        turn_factor = abs(self.pole_angle) / 40.0
        self.slip_rate = 0.08 + turn_factor * 0.3 + random.uniform(-0.05, 0.05)

        if random.random() < 0.05:
            self.slip_rate = random.uniform(0.7, 0.95)
        elif random.random() < 0.03:
            self.slip_rate = random.uniform(0.01, 0.04)

        self.slip_rate = max(0.0, min(1.0, self.slip_rate))

        self.roll_angle = turn_factor * 15.0 + random.uniform(-2, 2)

        if random.random() < 0.08:
            self.roll_angle = random.uniform(22, 30)

        self.roll_angle = max(-35, min(35, self.roll_angle))

        self.friction_coeff = 0.65 + random.uniform(-0.15, 0.15)
        self.friction_coeff = max(0.2, min(0.95, self.friction_coeff))

    def get_data(self) -> dict:
        return {
            "vehicle_id": self.vehicle_id,
            "pole_angle": round(self.pole_angle, 3),
            "slip_rate": round(self.slip_rate, 4),
            "roll_angle": round(self.roll_angle, 3),
            "friction_coeff": round(self.friction_coeff, 4),
            "timestamp": int(time.time())
        }

    def send_data(self) -> bool:
        data = self.get_data()
        try:
            response = requests.post(f"{API_URL}/api/sensor/data", json=data, timeout=5)
            if response.status_code == 200:
                print(f"[{self.vehicle_id}] 数据已发送 - 转角: {data['pole_angle']:.1f}° "
                      f"滑移率: {data['slip_rate']:.3f} 侧倾角: {data['roll_angle']:.1f}° "
                      f"摩擦系数: {data['friction_coeff']:.3f}")
                return True
            else:
                print(f"[{self.vehicle_id}] 发送失败: {response.status_code} - {response.text}")
                return False
        except Exception as e:
            print(f"[{self.vehicle_id}] 连接错误: {e}")
            return False


def main():
    vehicle_ids = ["chariot-qin-001", "chariot-han-002", "chariot-zhou-003"]
    simulators = [ChariotSensorSimulator(vid, base_speed=random.uniform(4, 7)) for vid in vehicle_ids]

    print("=" * 60)
    print("古代双辕车传感器模拟器")
    print("=" * 60)
    print(f"目标API: {API_URL}")
    print(f"车辆数量: {len(vehicle_ids)}")
    print(f"上报间隔: 60秒")
    print("=" * 60)
    print()

    interval = 60
    print("首次发送数据...")

    for sim in simulators:
        sim.send_data()

    print(f"\n进入循环模式，每 {interval} 秒上报一次...")
    print("按 Ctrl+C 停止\n")

    try:
        while True:
            time.sleep(interval)
            for sim in simulators:
                sim.update(interval)
                sim.send_data()
    except KeyboardInterrupt:
        print("\n\n模拟器已停止。")


if __name__ == "__main__":
    main()
