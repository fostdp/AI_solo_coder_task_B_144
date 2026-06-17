import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from influxdb_client import InfluxDBClient, BucketRetentionRules
from config.settings import INFLUXDB_URL, INFLUXDB_TOKEN, INFLUXDB_ORG, INFLUXDB_BUCKET


def init_influxdb():
    client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)

    buckets_api = client.buckets_api()
    orgs_api = client.organizations_api()

    try:
        orgs = orgs_api.find_organizations()
        org_names = [org.name for org in orgs]
        if INFLUXDB_ORG not in org_names:
            print(f"创建组织: {INFLUXDB_ORG}")
            org = orgs_api.create_organization(name=INFLUXDB_ORG)
        else:
            org = next(o for o in orgs if o.name == INFLUXDB_ORG)
            print(f"组织已存在: {INFLUXDB_ORG}")
    except Exception as e:
        print(f"组织操作失败: {e}")
        org = None

    try:
        buckets = buckets_api.find_buckets()
        bucket_names = [b.name for b in buckets.buckets]
        if INFLUXDB_BUCKET not in bucket_names:
            print(f"创建Bucket: {INFLUXDB_BUCKET}")
            retention_rules = BucketRetentionRules(type="expire", every_seconds=0)
            buckets_api.create_bucket(
                bucket_name=INFLUXDB_BUCKET,
                org=INFLUXDB_ORG,
                retention_rules=retention_rules
            )
        else:
            print(f"Bucket已存在: {INFLUXDB_BUCKET}")
    except Exception as e:
        print(f"Bucket操作失败: {e}")

    print("\nInfluxDB 初始化完成！")
    print(f"  URL: {INFLUXDB_URL}")
    print(f"  组织: {INFLUXDB_ORG}")
    print(f"  Bucket: {INFLUXDB_BUCKET}")
    print(f"\n测量数据(Measurement)说明:")
    print(f"  - chariot_sensor: 传感器原始数据")
    print(f"    Tags: vehicle_id, sensor_id")
    print(f"    Fields: pole_angle, slip_rate, roll_angle, friction_coeff")
    print(f"  - steering_analysis: 转向分析结果")
    print(f"    Tags: vehicle_id")
    print(f"    Fields: turning_radius, inner_wheel_angle, outer_wheel_angle, wheel_speed_diff")
    print(f"  - stability_analysis: 稳定性分析结果")
    print(f"    Tags: vehicle_id")
    print(f"    Fields: yaw_rate, roll_center_height, rollover_risk, lateral_acceleration")

    client.close()


if __name__ == "__main__":
    init_influxdb()
