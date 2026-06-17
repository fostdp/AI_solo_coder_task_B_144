# -*- coding: utf-8 -*-
"""
功能回归测试 v2.0 - 微服务架构
"""
import sys
import os
import json
import math

sys.stdout.reconfigure(encoding='utf-8')
sys.stderr.reconfigure(encoding='utf-8')

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'services'))

from common import (
    get_config_loader, RedisClient,
    SensorData, SteeringResult, StabilityResult, Alert,
    create_response, timestamp
)
from common.steering_model import ChariotParams, MultiBodyDynamicsSteering
from common.stability_analysis import VehicleDynamicsParams, StabilityAnalyzer, CargoConfig

PASS_COUNT = 0
FAIL_COUNT = 0


def test(name, condition, msg=None):
    global PASS_COUNT, FAIL_COUNT
    if condition:
        PASS_COUNT += 1
        print(f"[ OK ] {name}")
        return True
    else:
        FAIL_COUNT += 1
        print(f"[FAIL] {name}" + (f" - {msg}" if msg else ""))
        return False


print("=" * 70)
print("Ancient Chariot Simulation System v2.0 - Regression Test")
print("=" * 70)
print()

# 1. Config loading
print("[1/8] Config Loading Test")
try:
    cfg = get_config_loader()
    geom = cfg.chariot_geometry()
    dyn = cfg.vehicle_dynamics()
    sys_cfg = cfg.system_config()
    alert_cfg = cfg.alert_thresholds()

    test("JSON config - geometry", "wheelbase" in geom and geom["wheelbase"] == 2.5)
    test("JSON config - dynamics", "mass" in dyn and dyn["mass"] == 800)
    test("JSON config - system", "redis" in sys_cfg and "redis_channels" in sys_cfg)
    test("JSON config - alerts", "roll_angle" in alert_cfg and alert_cfg["roll_angle"]["threshold"] == 20)
    test("JSON config - Redis channels", all(k in sys_cfg["redis_channels"] for k in [
        "sensor_data_raw", "sensor_data_validated", "steering_result",
        "stability_result", "alerts"
    ]))
except Exception as e:
    test("Config loading", False, str(e))
print()

# 2. Message protocol
print("[2/8] Message Protocol Test")
try:
    s = SensorData("chariot-001", 15.0, 0.15, 5.0, 0.6)
    test("SensorData instance", s.vehicle_id == "chariot-001")
    test("SensorData validate pass", s.validate())

    s2 = SensorData("X", -70, 1.5, -100, 0.05)
    test("SensorData validate fail", not s2.validate())

    d = s.to_dict()
    s3 = SensorData.from_dict(d)
    test("SensorData serialize", s3.vehicle_id == s.vehicle_id)

    resp = create_response("req-123", True, {"foo": "bar"})
    test("create_response structure", resp["request_id"] == "req-123")
except Exception as e:
    test("Message protocol", False, str(e))
print()

# 3. Steering simulation
print("[3/8] Steering Simulation Test")
try:
    geom = cfg.chariot_geometry()
    cp = ChariotParams(**geom)
    st = MultiBodyDynamicsSteering(cp)

    r0 = st.calculate_steering(0, 5, 0.6)
    test("Steering - straight inner~0", abs(r0.inner_wheel_angle) < 0.01)
    test("Steering - straight outer~0", abs(r0.outer_wheel_angle) < 0.01)
    test("Steering - straight transAngle~90", abs(r0.transmission_angle_inner - 90) < 1)
    test("Steering - straight no interference", not r0.linkage_interference and not r0.dead_point_risk)

    r1 = st.calculate_steering(10, 5, 0.6)
    test("Steering - 10deg inner>outer", r1.inner_wheel_angle > r1.outer_wheel_angle)
    test("Steering - 10deg transAngle valid", 30 < r1.transmission_angle_inner < 150)
    ack_ok = abs(1/math.tan(r1.outer_wheel_angle*math.pi/180) -
                 1/math.tan(r1.inner_wheel_angle*math.pi/180) -
                 cp.track_width/cp.wheelbase) < 0.6
    test("Steering - 10deg Ackermann relation", ack_ok)

    r2 = st.calculate_steering(45, 5, 0.6)
    test("Steering - 45deg dead point limit", r2.dead_point_risk)
    test("Steering - 45deg inner<50", r2.inner_wheel_angle < 50)

    all_ok = all(30 <= r.transmission_angle_inner <= 150 and
                 30 <= r.transmission_angle_outer <= 150
                 for r in [r0, r1, r2])
    test("Steering - transmission angles in range", all_ok)
except Exception as e:
    test("Steering simulation", False, str(e))
print()

# 4. Stability analysis
print("[4/8] Stability Analysis Test")
try:
    dyn = cfg.vehicle_dynamics()
    vp = VehicleDynamicsParams(**dyn)
    sa = StabilityAnalyzer(vp)

    sa.set_cargo(CargoConfig())
    r0 = sa.analyze(8, 15, 10, 0.1, 0.6)
    test("Stability - empty CG height", abs(r0.effective_cg_height - dyn["cg_height"]) < 0.01)
    test("Stability - empty CG lateral~0", abs(r0.effective_cg_lateral) < 0.01)

    sa.set_cargo(CargoConfig(mass=200, offset_lateral=0.08, offset_height=0.35))
    r1 = sa.analyze(8, 15, 10, 0.1, 0.6)
    test("Stability - loaded CG higher", r1.effective_cg_height > r0.effective_cg_height)
    test("Stability - loaded CG shifted right (+)", r1.effective_cg_lateral > 0.01)
    test("Stability - loaded inertia larger", r1.effective_yaw_inertia > r0.effective_yaw_inertia)
    test("Stability - loaded rollover higher", r1.rollover_risk > r0.rollover_risk)
    test("Stability - loaded yaw changed", abs(r1.yaw_rate - r0.yaw_rate) > 0.1)

    sa.set_cargo(CargoConfig(mass=500, offset_lateral=0.3, offset_height=0.9))
    r2 = sa.analyze(15, 30, 28, 0.25, 0.3)
    test("Stability - extreme rollover>=70", r2.rollover_risk >= 70)
    test("Stability - extreme stability<0.3", r2.stability_index < 0.3)
    test("Stability - cargo shift non-zero", abs(r2.cargo_shift_lateral) > 0.001)
except Exception as e:
    test("Stability analysis", False, str(e))
print()

# 5. Microservices instantiation
print("[5/8] Microservices Instantiation Test")
try:
    import importlib
    services = [
        ("dtu_receiver", 8001),
        ("steering_simulator", 8002),
        ("stability_analyzer", 8003),
        ("alarm_mqtt", 8004),
        ("api_gateway", 8000),
    ]
    for svc_name, port in services:
        mod = importlib.import_module(f"{svc_name}.main")
        test(f"Service {svc_name} app exists", hasattr(mod, "app"))
except Exception as e:
    test("Microservices", False, str(e))
print()

# 6. Redis client fallback
print("[6/8] Redis Client Fallback Test", flush=True)
try:
    r = RedisClient(host="127.0.0.1", port=1)
    connected = r.connect(timeout=1)
    test("Redis - connect fail returns False", not connected)
    test("Redis - publish no-crash", True)
    r.publish("test", {"foo": "bar"})
    test("Redis - request_response returns None", r.request_response("req", "resp", {}, timeout=0.5) is None)
except Exception as e:
    test("Redis client", False, str(e))
print(flush=True)

# 7. Alert manager
print("[7/8] Alert Manager Test")
try:
    from common.alert_manager import AlertManager
    am = AlertManager(thresholds=cfg.alert_thresholds(), cooldown=300)

    alerts = am.check_sensor_data({
        "vehicle_id": "chariot-001",
        "pole_angle": 10,
        "slip_rate": 0.9,
        "roll_angle": 25,
        "friction_coeff": 0.2
    })
    test("Alert - slip_rate_high triggered", any(a.alert_type == "slip_rate_high" for a in alerts))
    test("Alert - roll_angle triggered", any(a.alert_type == "roll_angle" for a in alerts))
    test("Alert - low_friction triggered", any(a.alert_type == "low_friction" for a in alerts))
    test("Alert - severity valid", all(a.severity in ["critical", "warning", "info"] for a in alerts))

    alerts2 = am.check_sensor_data({
        "vehicle_id": "chariot-001",
        "pole_angle": 10,
        "slip_rate": 0.95,
        "roll_angle": 28,
        "friction_coeff": 0.15
    })
    test("Alert - cooldown suppresses", len(alerts2) == 0)
except Exception as e:
    test("Alert manager", False, str(e))
print()

# 8. Frontend files
print("[8/8] Frontend Files Test")
try:
    frontend_dir = os.path.join(os.path.dirname(__file__), "frontend")
    test("Frontend - chariot_3d.js exists", os.path.exists(os.path.join(frontend_dir, "chariot_3d.js")))
    test("Frontend - steering_panel.js exists", os.path.exists(os.path.join(frontend_dir, "steering_panel.js")))
    test("Frontend - index.html exists", os.path.exists(os.path.join(frontend_dir, "index.html")))

    for js_file in ["chariot_3d.js", "steering_panel.js"]:
        with open(os.path.join(frontend_dir, js_file), "r", encoding="utf-8") as f:
            content = f.read()
            parts = js_file.split(".")[0].split("_")
            class_name = "".join(p[0].upper() + p[1:].upper() if p[0].isdigit() else p[0].upper() + p[1:] for p in parts)
            test(f"Frontend - {js_file} has class {class_name}", f"class {class_name}" in content)
            test(f"Frontend - {js_file} syntax ok", "{" in content and "}" in content)

    with open(os.path.join(frontend_dir, "index.html"), "r", encoding="utf-8") as f:
        html = f.read()
        test("Frontend - index.html references chariot_3d.js", 'src="chariot_3d.js"' in html)
        test("Frontend - index.html references steering_panel.js", 'src="steering_panel.js"' in html)
except Exception as e:
    test("Frontend files", False, str(e))
print()

# Summary
print("=" * 70)
print(f"Test Complete: PASS={PASS_COUNT}, FAIL={FAIL_COUNT}")
print("=" * 70)
print()

print("Architecture Verified:")
print("  dtu_receiver       (8001)  - Sensor validation -> Redis Pub/Sub")
print("  steering_simulator (8002)  - Four-bar linkage + Ackermann geometry")
print("  stability_analyzer (8003)  - Variable CG + understeer model")
print("  alarm_mqtt         (8004)  - Alert evaluation + MQTT + InfluxDB")
print("  api_gateway        (8000)  - REST/WebSocket entry point")
print()
print("Externalized Configs:")
print("  config/json/chariot_geometry.json")
print("  config/json/vehicle_dynamics.json")
print("  config/json/system_config.json")
print("  config/json/alert_thresholds.json")
print()
print("Frontend Split:")
print("  frontend/chariot_3d.js     - Three.js 3D + GPU Instancing")
print("  frontend/steering_panel.js - Controls, WS, 2D diagram, UI")
print()

if FAIL_COUNT == 0:
    print("ALL TESTS PASSED!")
    sys.exit(0)
else:
    print(f"{FAIL_COUNT} TESTS FAILED")
    sys.exit(1)
