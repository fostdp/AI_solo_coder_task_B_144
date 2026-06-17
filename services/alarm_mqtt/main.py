import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from typing import Dict, Any, List, Optional
from fastapi import FastAPI
import uvicorn

from common import (
    get_config_loader, RedisClient, SensorData, SteeringResult,
    StabilityResult, Alert, timestamp
)
from common.alert_manager import AlertManager


try:
    from influxdb_client import InfluxDBClient, Point, WriteOptions
    from influxdb_client.client.write_api import SYNCHRONOUS
    INFLUX_AVAILABLE = True
except ImportError:
    INFLUX_AVAILABLE = False

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False


config = get_config_loader()
app = FastAPI(title="Alarm & MQTT Service", version="2.0.0")

redis: RedisClient = None
alert_manager: AlertManager = None
influx_client: Optional[Any] = None
mqtt_client: Optional[Any] = None
write_api: Optional[Any] = None

channels: Dict[str, str] = {}
sensor_cache: Dict[str, SensorData] = {}


@app.on_event("startup")
async def startup():
    global redis, alert_manager, channels
    global influx_client, mqtt_client, write_api

    sys_cfg = config.system_config()
    alert_cfg = config.alert_thresholds()
    channels = sys_cfg['redis_channels']

    redis = RedisClient(**sys_cfg['redis'])
    redis.connect()

    alert_manager = AlertManager(
        thresholds=alert_cfg,
        cooldown=alert_cfg['cooldown_seconds']
    )

    # 初始化InfluxDB
    if INFLUX_AVAILABLE:
        try:
            influx_cfg = sys_cfg['influxdb']
            influx_client = InfluxDBClient(
                url=influx_cfg['url'],
                token=influx_cfg['token'],
                org=influx_cfg['org']
            )
            write_api = influx_client.write_api(write_options=SYNCHRONOUS)
            print(f"[alarm_mqtt] InfluxDB连接成功: {influx_cfg['url']}")
        except Exception as e:
            print(f"[alarm_mqtt] InfluxDB连接失败: {e}")

    # 初始化MQTT
    if MQTT_AVAILABLE:
        try:
            mqtt_cfg = sys_cfg['mqtt']
            mqtt_client = mqtt.Client(client_id="alarm_mqtt_service")
            mqtt_client.connect(mqtt_cfg['broker'], mqtt_cfg['port'], keepalive=60)
            mqtt_client.loop_start()
            print(f"[alarm_mqtt] MQTT连接成功: {mqtt_cfg['broker']}:{mqtt_cfg['port']}")
        except Exception as e:
            print(f"[alarm_mqtt] MQTT连接失败: {e}")

    # 订阅通道
    redis.subscribe(channels['sensor_data_validated'], on_sensor_data)
    redis.subscribe(channels['steering_result'], on_steering_result)
    redis.subscribe(channels['stability_result'], on_stability_result)

    print(f"[alarm_mqtt] 启动完成, 订阅: sensor/steering/stability")


def on_sensor_data(data: Dict[str, Any]) -> None:
    try:
        sensor = SensorData.from_dict(data)
        sensor_cache[sensor.vehicle_id] = sensor
        save_sensor_to_influx(sensor)
        publish_mqtt_sensor(sensor)
    except Exception as e:
        print(f"[alarm_mqtt] 处理传感器数据失败: {e}")


def on_steering_result(data: Dict[str, Any]) -> None:
    try:
        steering = SteeringResult.from_dict(data)
        save_steering_to_influx(steering)
    except Exception as e:
        print(f"[alarm_mqtt] 处理转向结果失败: {e}")


def on_stability_result(data: Dict[str, Any]) -> None:
    try:
        stability = StabilityResult.from_dict(data)
        sensor = sensor_cache.get(stability.vehicle_id)
        save_stability_to_influx(stability)

        # 告警检查
        if sensor:
            alerts = alert_manager.check_sensor_data(sensor.to_dict())
            alerts += alert_manager.check_stability(stability.to_dict())
        else:
            alerts = alert_manager.check_stability(stability.to_dict())

        # 推送告警
        for alert in alerts:
            redis.publish(channels['alerts'], alert.to_dict())
            publish_mqtt_alert(alert)
            print(f"[alarm] 车辆={alert.vehicle_id} {alert.alert_type}={alert.value:.2f} {alert.severity}: {alert.message}")

        # 推送完整数据到MQTT
        publish_mqtt_full_data(stability, sensor)

    except Exception as e:
        print(f"[alarm_mqtt] 处理稳定性结果失败: {e}")


def save_sensor_to_influx(sensor: SensorData) -> None:
    if not write_api:
        return
    try:
        influx_cfg = config.system_config()['influxdb']
        point = Point("chariot_sensor") \
            .tag("vehicle_id", sensor.vehicle_id) \
            .field("pole_angle", sensor.pole_angle) \
            .field("slip_rate", sensor.slip_rate) \
            .field("roll_angle", sensor.roll_angle) \
            .field("friction_coeff", sensor.friction_coeff) \
            .time(int(sensor.timestamp))
        write_api.write(bucket=influx_cfg['bucket'], org=influx_cfg['org'], record=point)
    except Exception as e:
        print(f"[influx] 写入sensor失败: {e}")


def save_steering_to_influx(steering: SteeringResult) -> None:
    if not write_api:
        return
    try:
        influx_cfg = config.system_config()['influxdb']
        point = Point("steering_analysis") \
            .tag("vehicle_id", steering.vehicle_id) \
            .field("turning_radius", steering.turning_radius) \
            .field("inner_wheel_angle", steering.inner_wheel_angle) \
            .field("outer_wheel_angle", steering.outer_wheel_angle) \
            .field("wheel_speed_diff", steering.wheel_speed_diff) \
            .field("ackermann_error", steering.ackermann_error) \
            .field("transmission_angle_inner", steering.transmission_angle_inner) \
            .field("transmission_angle_outer", steering.transmission_angle_outer) \
            .field("linkage_interference", int(steering.linkage_interference)) \
            .field("dead_point_risk", int(steering.dead_point_risk)) \
            .time(int(steering.timestamp))
        write_api.write(bucket=influx_cfg['bucket'], org=influx_cfg['org'], record=point)
    except Exception as e:
        print(f"[influx] 写入steering失败: {e}")


def save_stability_to_influx(stability: StabilityResult) -> None:
    if not write_api:
        return
    try:
        influx_cfg = config.system_config()['influxdb']
        point = Point("stability_analysis") \
            .tag("vehicle_id", stability.vehicle_id) \
            .field("yaw_rate", stability.yaw_rate) \
            .field("roll_center_height", stability.roll_center_height) \
            .field("rollover_risk", stability.rollover_risk) \
            .field("lateral_acceleration", stability.lateral_acceleration) \
            .field("stability_index", stability.stability_index) \
            .field("critical_speed", stability.critical_speed) \
            .field("effective_cg_height", stability.effective_cg_height) \
            .field("effective_cg_lateral", stability.effective_cg_lateral) \
            .field("effective_cg_longitudinal", stability.effective_cg_longitudinal) \
            .field("effective_yaw_inertia", stability.effective_yaw_inertia) \
            .field("cargo_shift_lateral", stability.cargo_shift_lateral) \
            .field("cargo_shift_vertical", stability.cargo_shift_vertical) \
            .time(int(stability.timestamp))
        write_api.write(bucket=influx_cfg['bucket'], org=influx_cfg['org'], record=point)
    except Exception as e:
        print(f"[influx] 写入stability失败: {e}")


def publish_mqtt_sensor(sensor: SensorData) -> None:
    if not mqtt_client:
        return
    try:
        import json
        mqtt_cfg = config.system_config()['mqtt']
        topic = f"{mqtt_cfg['topic_data']}/{sensor.vehicle_id}/sensor"
        mqtt_client.publish(topic, json.dumps(sensor.to_dict(), ensure_ascii=False), qos=0)
    except Exception as e:
        print(f"[mqtt] 推送sensor失败: {e}")


def publish_mqtt_alert(alert: Alert) -> None:
    if not mqtt_client:
        return
    try:
        import json
        mqtt_cfg = config.system_config()['mqtt']
        topic = f"{mqtt_cfg['topic_alert']}/{alert.vehicle_id}/{alert.alert_type}"
        mqtt_client.publish(topic, json.dumps(alert.to_dict(), ensure_ascii=False), qos=1)
    except Exception as e:
        print(f"[mqtt] 推送alert失败: {e}")


def publish_mqtt_full_data(stability: StabilityResult, sensor: Optional[SensorData]) -> None:
    if not mqtt_client:
        return
    try:
        import json
        mqtt_cfg = config.system_config()['mqtt']
        topic = f"{mqtt_cfg['topic_data']}/{stability.vehicle_id}/full"
        data = {
            "timestamp": stability.timestamp,
            "vehicle_id": stability.vehicle_id,
            "sensor": sensor.to_dict() if sensor else {},
            "stability": stability.to_dict(),
            "alerts": []
        }
        mqtt_client.publish(topic, json.dumps(data, ensure_ascii=False), qos=0)
    except Exception as e:
        print(f"[mqtt] 推送full数据失败: {e}")


@app.get("/health")
async def health_check():
    return {
        "status": "ok",
        "service": "alarm_mqtt",
        "influx_available": INFLUX_AVAILABLE and influx_client is not None,
        "mqtt_available": MQTT_AVAILABLE and mqtt_client is not None
    }


if __name__ == "__main__":
    sys_cfg = config.system_config()
    uvicorn.run(app, host=sys_cfg['fastapi']['host'], port=8004)
