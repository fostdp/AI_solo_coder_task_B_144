#!/bin/bash

set -e

echo "=== InfluxDB 初始化开始 ==="

until influx ping -t "${DOCKER_INFLUXDB_INIT_ADMIN_TOKEN}" > /dev/null 2>&1; do
    echo "等待 InfluxDB 启动..."
    sleep 2
done

echo "InfluxDB 已连接，开始配置..."

ORG="${DOCKER_INFLUXDB_INIT_ORG}"
BUCKET="${DOCKER_INFLUXDB_INIT_BUCKET}"
TOKEN="${DOCKER_INFLUXDB_INIT_ADMIN_TOKEN}"

echo "创建保留策略（RP）..."

influx bucket create \
    --name "${BUCKET}_downsampled_1h" \
    --org "${ORG}" \
    --retention 168h \
    --token "${TOKEN}" 2>/dev/null || echo "1小时降采样桶已存在"

influx bucket create \
    --name "${BUCKET}_downsampled_1d" \
    --org "${ORG}" \
    --retention 8760h \
    --token "${TOKEN}" 2>/dev/null || echo "1天降采样桶已存在"

echo "创建降采样任务（Tasks）..."

TASK_1H=$(cat <<'EOF'
option task = {name: "Downsample to 1h", every: 1h}

data = from(bucket: "chariot_data")
    |> range(start: -1h)
    |> filter(fn: (r) => r._measurement == "chariot_sensor")
    |> aggregateWindow(every: 1h, fn: mean, createEmpty: false)
    |> to(bucket: "chariot_data_downsampled_1h", org: "chariot-research")

steering = from(bucket: "chariot_data")
    |> range(start: -1h)
    |> filter(fn: (r) => r._measurement == "steering_analysis")
    |> aggregateWindow(every: 1h, fn: mean, createEmpty: false)
    |> to(bucket: "chariot_data_downsampled_1h", org: "chariot-research")

stability = from(bucket: "chariot_data")
    |> range(start: -1h)
    |> filter(fn: (r) => r._measurement == "stability_analysis")
    |> aggregateWindow(every: 1h, fn: mean, createEmpty: false)
    |> to(bucket: "chariot_data_downsampled_1h", org: "chariot-research")
EOF
)

TASK_1D=$(cat <<'EOF'
option task = {name: "Downsample to 1d", every: 1d}

data = from(bucket: "chariot_data_downsampled_1h")
    |> range(start: -1d)
    |> aggregateWindow(every: 1d, fn: mean, createEmpty: false)
    |> to(bucket: "chariot_data_downsampled_1d", org: "chariot-research")
EOF
)

influx task create --org "${ORG}" --token "${TOKEN}" --flux "${TASK_1H}" 2>/dev/null || echo "1小时降采样任务已存在"
influx task create --org "${ORG}" --token "${TOKEN}" --flux "${TASK_1D}" 2>/dev/null || echo "1天降采样任务已存在"

echo "设置原始数据保留策略（7天）..."

influx bucket update \
    --name "${BUCKET}" \
    --org "${ORG}" \
    --retention 168h \
    --token "${TOKEN}" 2>/dev/null || echo "保留策略更新完成"

echo "=== InfluxDB 初始化完成 ==="
echo "数据保留策略："
echo "  - 原始数据（1分钟级）：保留7天"
echo "  - 1小时降采样：保留7天"
echo "  - 1天降采样：保留365天"
echo "降采样任务："
echo "  - 每小时聚合一次到 _downsampled_1h 桶"
echo "  - 每天聚合一次到 _downsampled_1d 桶"
