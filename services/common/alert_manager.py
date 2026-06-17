import json
import time
import threading
from typing import Optional, Dict, List
from dataclasses import dataclass, asdict
from datetime import datetime

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False


@dataclass
class Alert:
    vehicle_id: str
    alert_type: str
    severity: str
    value: float
    threshold: float
    message: str
    timestamp: float
    acknowledged: bool = False


class AlertManager:
    def __init__(self, broker: str = "localhost", port: int = 1883,
                 topic_alert: str = "chariot/alerts",
                 topic_data: str = "chariot/data",
                 roll_threshold: float = 20.0,
                 slip_low: float = 0.05,
                 slip_high: float = 0.8,
                 thresholds: Optional[Dict[str, Any]] = None,
                 cooldown: int = 300):
        self.broker = broker
        self.port = port
        self.topic_alert = topic_alert
        self.topic_data = topic_data

        if thresholds:
            self.roll_threshold = thresholds.get("roll_angle", {}).get("threshold", roll_threshold)
            self.slip_high = thresholds.get("slip_rate_high", {}).get("threshold", slip_high)
            self.slip_low = thresholds.get("slip_rate_low", {}).get("threshold", slip_low)
            self.low_friction_threshold = thresholds.get("low_friction", {}).get("threshold", 0.3)
            self.cooldown_period = thresholds.get("cooldown_seconds", cooldown)
        else:
            self.roll_threshold = roll_threshold
            self.slip_low = slip_low
            self.slip_high = slip_high
            self.low_friction_threshold = 0.3
            self.cooldown_period = cooldown

        self.alerts: List[Alert] = []
        self.alert_history: Dict[str, Dict] = {}
        self._lock = threading.Lock()
        self.mqtt_client: Optional[mqtt.Client] = None

    def connect_mqtt(self) -> bool:
        if not MQTT_AVAILABLE:
            print("警告: paho-mqtt 未安装，MQTT功能不可用")
            return False

        try:
            self.mqtt_client = mqtt.Client(client_id="chariot_alert_manager")
            self.mqtt_client.on_connect = self._on_connect
            self.mqtt_client.on_disconnect = self._on_disconnect
            self.mqtt_client.connect(self.broker, self.port, keepalive=60)
            self.mqtt_client.loop_start()
            return True
        except Exception as e:
            print(f"MQTT连接失败: {e}")
            return False

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("MQTT连接成功")
        else:
            print(f"MQTT连接失败，错误码: {rc}")

    def _on_disconnect(self, client, userdata, rc):
        print(f"MQTT断开连接，错误码: {rc}")

    def _is_in_cooldown(self, vehicle_id: str, alert_type: str) -> bool:
        key = f"{vehicle_id}_{alert_type}"
        if key in self.alert_history:
            last_time = self.alert_history[key]["last_triggered"]
            if time.time() - last_time < self.cooldown_period:
                return True
        return False

    def _update_cooldown(self, vehicle_id: str, alert_type: str):
        key = f"{vehicle_id}_{alert_type}"
        if key not in self.alert_history:
            self.alert_history[key] = {"count": 0, "last_triggered": 0}
        self.alert_history[key]["count"] += 1
        self.alert_history[key]["last_triggered"] = time.time()

    def check_sensor_data(self, *args, **kwargs) -> List[Alert]:
        if len(args) == 1 and isinstance(args[0], dict):
            data = args[0]
            vehicle_id = data.get("vehicle_id", "")
            pole_angle = data.get("pole_angle", 0)
            slip_rate = data.get("slip_rate", 0)
            roll_angle = data.get("roll_angle", 0)
            friction_coeff = data.get("friction_coeff", 0)
        else:
            vehicle_id = kwargs.get("vehicle_id", args[0] if len(args) > 0 else "")
            pole_angle = kwargs.get("pole_angle", args[1] if len(args) > 1 else 0)
            slip_rate = kwargs.get("slip_rate", args[2] if len(args) > 2 else 0)
            roll_angle = kwargs.get("roll_angle", args[3] if len(args) > 3 else 0)
            friction_coeff = kwargs.get("friction_coeff", args[4] if len(args) > 4 else 0)

        triggered_alerts = []

        if abs(roll_angle) > self.roll_threshold:
            if not self._is_in_cooldown(vehicle_id, "roll_angle"):
                severity = "critical" if abs(roll_angle) > self.roll_threshold * 1.3 else "warning"
                alert = Alert(
                    vehicle_id=vehicle_id,
                    alert_type="roll_angle",
                    severity=severity,
                    value=roll_angle,
                    threshold=self.roll_threshold,
                    message=f"车身侧倾角 {roll_angle:.1f}° 超过阈值 {self.roll_threshold}°",
                    timestamp=time.time()
                )
                triggered_alerts.append(alert)
                self._update_cooldown(vehicle_id, "roll_angle")

        if slip_rate > self.slip_high:
            if not self._is_in_cooldown(vehicle_id, "slip_rate_high"):
                alert = Alert(
                    vehicle_id=vehicle_id,
                    alert_type="slip_rate_high",
                    severity="warning",
                    value=slip_rate,
                    threshold=self.slip_high,
                    message=f"车轮滑移率 {slip_rate:.3f} 超过上限 {self.slip_high}，可能打滑",
                    timestamp=time.time()
                )
                triggered_alerts.append(alert)
                self._update_cooldown(vehicle_id, "slip_rate_high")

        if slip_rate < self.slip_low:
            if not self._is_in_cooldown(vehicle_id, "slip_rate_low"):
                alert = Alert(
                    vehicle_id=vehicle_id,
                    alert_type="slip_rate_low",
                    severity="info",
                    value=slip_rate,
                    threshold=self.slip_low,
                    message=f"车轮滑移率 {slip_rate:.3f} 低于下限 {self.slip_low}，传感器可能异常",
                    timestamp=time.time()
                )
                triggered_alerts.append(alert)
                self._update_cooldown(vehicle_id, "slip_rate_low")

        if friction_coeff < 0.3:
            if not self._is_in_cooldown(vehicle_id, "low_friction"):
                alert = Alert(
                    vehicle_id=vehicle_id,
                    alert_type="low_friction",
                    severity="warning",
                    value=friction_coeff,
                    threshold=0.3,
                    message=f"路面摩擦系数 {friction_coeff:.3f} 过低，易打滑",
                    timestamp=time.time()
                )
                triggered_alerts.append(alert)
                self._update_cooldown(vehicle_id, "low_friction")

        if triggered_alerts:
            with self._lock:
                self.alerts.extend(triggered_alerts)
                self._publish_alerts(triggered_alerts)

        return triggered_alerts

    def _publish_alerts(self, alerts: List[Alert]):
        if self.mqtt_client is None:
            return

        try:
            for alert in alerts:
                payload = json.dumps({
                    "vehicle_id": alert.vehicle_id,
                    "alert_type": alert.alert_type,
                    "severity": alert.severity,
                    "value": alert.value,
                    "threshold": alert.threshold,
                    "message": alert.message,
                    "timestamp": alert.timestamp,
                    "datetime": datetime.fromtimestamp(alert.timestamp).isoformat()
                }, ensure_ascii=False)

                topic = f"{self.topic_alert}/{alert.vehicle_id}/{alert.alert_type}"
                self.mqtt_client.publish(topic, payload, qos=1)
                print(f"[MQTT告警] {alert.vehicle_id} - {alert.message}")
        except Exception as e:
            print(f"发布告警失败: {e}")

    def publish_data(self, vehicle_id: str, data: dict):
        if self.mqtt_client is None:
            return

        try:
            payload = json.dumps(data, ensure_ascii=False)
            topic = f"{self.topic_data}/{vehicle_id}"
            self.mqtt_client.publish(topic, payload, qos=0)
        except Exception as e:
            print(f"发布数据失败: {e}")

    def get_recent_alerts(self, vehicle_id: Optional[str] = None,
                          limit: int = 50) -> List[dict]:
        with self._lock:
            alerts = self.alerts.copy()

        if vehicle_id:
            alerts = [a for a in alerts if a.vehicle_id == vehicle_id]

        alerts.sort(key=lambda a: a.timestamp, reverse=True)
        alerts = alerts[:limit]

        return [
            {
                "vehicle_id": a.vehicle_id,
                "alert_type": a.alert_type,
                "severity": a.severity,
                "value": a.value,
                "threshold": a.threshold,
                "message": a.message,
                "timestamp": a.timestamp,
                "datetime": datetime.fromtimestamp(a.timestamp).isoformat()
            }
            for a in alerts
        ]

    def get_alert_stats(self) -> dict:
        with self._lock:
            alerts = self.alerts.copy()

        stats = {
            "total": len(alerts),
            "by_type": {},
            "by_severity": {},
            "by_vehicle": {}
        }

        for alert in alerts:
            stats["by_type"][alert.alert_type] = stats["by_type"].get(alert.alert_type, 0) + 1
            stats["by_severity"][alert.severity] = stats["by_severity"].get(alert.severity, 0) + 1
            stats["by_vehicle"][alert.vehicle_id] = stats["by_vehicle"].get(alert.vehicle_id, 0) + 1

        return stats

    def check_stability(self, *args, **kwargs) -> List[Alert]:
        if len(args) == 1 and isinstance(args[0], dict):
            data = args[0]
            vehicle_id = data.get("vehicle_id", "")
            rollover_risk = data.get("rollover_risk", 0.0)
            yaw_rate = data.get("yaw_rate", 0.0)
            stability_index = data.get("stability_index", 1.0)
        else:
            vehicle_id = kwargs.get("vehicle_id", args[0] if len(args) > 0 else "")
            rollover_risk = kwargs.get("rollover_risk", args[1] if len(args) > 1 else 0.0)
            yaw_rate = kwargs.get("yaw_rate", args[2] if len(args) > 2 else 0.0)
            stability_index = kwargs.get("stability_index", args[3] if len(args) > 3 else 1.0)

        triggered_alerts = []

        if rollover_risk > 70:
            if not self._is_in_cooldown(vehicle_id, "rollover_risk"):
                severity = "critical" if rollover_risk > 85 else "warning"
                alert = Alert(
                    vehicle_id=vehicle_id,
                    alert_type="rollover_risk",
                    severity=severity,
                    value=rollover_risk,
                    threshold=70.0,
                    message=f"侧翻风险 {rollover_risk:.1f}% 超过阈值 70%，立即采取措施",
                    timestamp=time.time()
                )
                triggered_alerts.append(alert)
                self._update_cooldown(vehicle_id, "rollover_risk")

        if stability_index < 0.3:
            if not self._is_in_cooldown(vehicle_id, "low_stability"):
                severity = "critical" if stability_index < 0.15 else "warning"
                alert = Alert(
                    vehicle_id=vehicle_id,
                    alert_type="low_stability",
                    severity=severity,
                    value=stability_index,
                    threshold=0.3,
                    message=f"稳定性指数 {stability_index:.2f} 低于阈值 0.3，行驶不稳定",
                    timestamp=time.time()
                )
                triggered_alerts.append(alert)
                self._update_cooldown(vehicle_id, "low_stability")

        if abs(yaw_rate) > 50:
            if not self._is_in_cooldown(vehicle_id, "high_yaw_rate"):
                severity = "warning"
                alert = Alert(
                    vehicle_id=vehicle_id,
                    alert_type="high_yaw_rate",
                    severity=severity,
                    value=yaw_rate,
                    threshold=50.0,
                    message=f"横摆角速度 {yaw_rate:.1f}°/s 过高，存在甩尾风险",
                    timestamp=time.time()
                )
                triggered_alerts.append(alert)
                self._update_cooldown(vehicle_id, "high_yaw_rate")

        if triggered_alerts:
            with self._lock:
                self.alerts.extend(triggered_alerts)
                self._publish_alerts(triggered_alerts)

        return triggered_alerts

    def close(self):
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
            self.mqtt_client = None
