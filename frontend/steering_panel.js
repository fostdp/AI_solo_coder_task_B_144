/* ============================================================
 * steering_panel.js - 控制面板与数据交互模块
 * 职责：WebSocket通信、数据面板更新、2D连杆示意图、手动操控
 * 依赖：chariot_3d.js (Chariot3D类), echarts.min.js (可选)
 * ============================================================ */

class SteeringPanel {
    constructor(containerId, chariot3d) {
        this.container = document.getElementById(containerId);
        this.chariot3d = chariot3d;
        this.ws = null;
        this.currentVehicleId = 'chariot-001';
        this.manualMode = false;
        this.manualPoleAngle = 0;
        this.manualSpeed = 5;
        this.systemParams = null;
        this.alerts = [];
        this.autoRefreshInterval = null;

        this._bindElements();
        this._bindEvents();
        this.connectWebSocket();
        this.startAutoRefresh();
    }

    _bindElements() {
        this.els = {
            poleAngle: document.getElementById('poleAngle'),
            slipRate: document.getElementById('slipRate'),
            rollAngle: document.getElementById('rollAngle'),
            frictionCoeff: document.getElementById('frictionCoeff'),
            innerWheelAngle: document.getElementById('innerWheelAngle'),
            outerWheelAngle: document.getElementById('outerWheelAngle'),
            turningRadius: document.getElementById('turningRadius'),
            wheelSpeedDiff: document.getElementById('wheelSpeedDiff'),
            yawRate: document.getElementById('yawRate'),
            rollCenter: document.getElementById('rollCenter'),
            rolloverRisk: document.getElementById('rolloverRisk'),
            lateralAccel: document.getElementById('lateralAccel'),
            stabilityIndex: document.getElementById('stabilityIndex'),
            alertList: document.getElementById('alertList'),
            vehicleSelect: document.getElementById('vehicleSelect'),
            transmissionAngleInner: document.getElementById('transmissionAngleInner'),
            transmissionAngleOuter: document.getElementById('transmissionAngleOuter'),
            linkageInterference: document.getElementById('linkageInterference'),
            deadPointRisk: document.getElementById('deadPointRisk'),
            effectiveCgHeight: document.getElementById('effectiveCgHeight'),
            effectiveCgLateral: document.getElementById('effectiveCgLateral'),
            effectiveYawInertia: document.getElementById('effectiveYawInertia'),
            cargoShiftLateral: document.getElementById('cargoShiftLateral'),
            cargoMass: document.getElementById('cargoMass'),
            cargoOffsetX: document.getElementById('cargoOffsetX'),
            cargoOffsetY: document.getElementById('cargoOffsetY'),
            cargoOffsetZ: document.getElementById('cargoOffsetZ'),
            manualPoleSlider: document.getElementById('manualPoleSlider'),
            manualSpeedSlider: document.getElementById('manualSpeedSlider'),
            manualModeToggle: document.getElementById('manualModeToggle')
        };
    }

    _bindEvents() {
        if (this.els.vehicleSelect) {
            this.els.vehicleSelect.addEventListener('change', (e) => {
                this.currentVehicleId = e.target.value;
                this.fetchLatestData();
            });
        }

        if (this.els.manualModeToggle) {
            this.els.manualModeToggle.addEventListener('change', (e) => {
                this.manualMode = e.target.checked;
                if (this.manualMode) {
                    this.runManualAnalysis();
                }
            });
        }

        if (this.els.manualPoleSlider) {
            this.els.manualPoleSlider.addEventListener('input', (e) => {
                this.manualPoleAngle = parseFloat(e.target.value);
                if (this.els.manualPoleSlider) {
                    document.getElementById('manualPoleValue').textContent = `${this.manualPoleAngle.toFixed(1)}°`;
                }
                if (this.manualMode) this.runManualAnalysis();
            });
        }

        if (this.els.manualSpeedSlider) {
            this.els.manualSpeedSlider.addEventListener('input', (e) => {
                this.manualSpeed = parseFloat(e.target.value);
                if (document.getElementById('manualSpeedValue')) {
                    document.getElementById('manualSpeedValue').textContent = `${this.manualSpeed.toFixed(1)} m/s`;
                }
                if (this.manualMode) this.runManualAnalysis();
            });
        }

        ['cargoMass', 'cargoOffsetX', 'cargoOffsetY', 'cargoOffsetZ'].forEach(id => {
            if (this.els[id]) {
                this.els[id].addEventListener('change', () => {
                    if (this.manualMode) this.runManualAnalysis();
                });
            }
        });

        document.querySelectorAll('.view-btn').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const view = e.target.dataset.view;
                this.chariot3d.setCameraView(view);
            });
        });

        document.getElementById('clearTrajectories')?.addEventListener('click', () => {
            this.chariot3d.clearTrajectories();
        });

        document.getElementById('fetchHistory')?.addEventListener('click', () => {
            this.fetchHistory();
        });
    }

    connectWebSocket() {
        const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${location.host}/ws/realtime`;

        try {
            this.ws = new WebSocket(wsUrl);
        } catch (e) {
            this.ws = new WebSocket(`ws://localhost:8000/ws/realtime`);
        }

        this.ws.onopen = () => {
            console.log('WebSocket连接成功');
            this.updateConnectionStatus(true);
        };

        this.ws.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                this._handleWebSocketMessage(data);
            } catch (e) {
                console.error('WebSocket消息解析失败:', e);
            }
        };

        this.ws.onerror = (e) => {
            console.error('WebSocket错误:', e);
            this.updateConnectionStatus(false);
        };

        this.ws.onclose = () => {
            console.log('WebSocket断开，3秒后重连...');
            this.updateConnectionStatus(false);
            setTimeout(() => this.connectWebSocket(), 3000);
        };
    }

    _handleWebSocketMessage(data) {
        switch (data.type) {
            case 'system_params':
                this.systemParams = data;
                this.displaySystemParams(data);
                break;
            case 'sensor_data':
                if (data.vehicle_id === this.currentVehicleId || this.els.vehicleSelect && !this.manualMode) {
                    this.displaySensorData(data);
                    if (data.vehicle_id === this.currentVehicleId && !this.manualMode) {
                        this.chariot3d.setVehicleSpeed(5);
                    }
                }
                this._updateVehicleList(data.vehicle_id);
                break;
            case 'steering_result':
                if (data.vehicle_id === this.currentVehicleId) {
                    this.displaySteeringData(data);
                    this.chariot3d.updateSteering(data.pole_angle_input || 0, data);
                    this.chariot3d.updateWheelRotation(
                        1 - data.wheel_speed_diff / 2,
                        1 + data.wheel_speed_diff / 2
                    );
                }
                break;
            case 'stability_result':
                if (data.vehicle_id === this.currentVehicleId) {
                    this.displayStabilityData(data);
                    this.chariot3d.setRolloverRisk(data.rollover_risk);
                    this.chariot3d.updateTrajectory(data, data.speed || 5);
                }
                break;
            case 'alert':
                this.addAlert(data);
                break;
            case 'snapshot':
                this._updateVehicleListFromSnapshot(data);
                break;
        }
    }

    displaySystemParams(params) {
        const geom = params.chariot_geometry || {};
        const dyn = params.vehicle_dynamics || {};

        document.getElementById('sysWheelbase').textContent = `${geom.wheelbase || 'N/A'} m`;
        document.getElementById('sysTrackWidth').textContent = `${geom.track_width || 'N/A'} m`;
        document.getElementById('sysWheelRadius').textContent = `${geom.wheel_radius || 'N/A'} m`;
        document.getElementById('sysPoleLength').textContent = `${geom.pole_length || 'N/A'} m`;
        document.getElementById('sysMass').textContent = `${dyn.mass || 'N/A'} kg`;
        document.getElementById('sysCgHeight').textContent = `${dyn.cg_height || 'N/A'} m`;
        document.getElementById('sysYawInertia').textContent = `${dyn.yaw_inertia || 'N/A'} kg·m²`;
    }

    displaySensorData(data) {
        this.els.poleAngle.textContent = `${data.pole_angle.toFixed(1)}°`;
        this.els.slipRate.textContent = `${(data.slip_rate * 100).toFixed(1)}%`;
        this.els.rollAngle.textContent = `${data.roll_angle.toFixed(1)}°`;
        this.els.frictionCoeff.textContent = data.friction_coeff.toFixed(2);

        if (data.roll_angle > 20) {
            this.els.rollAngle.classList.add('text-red-600', 'font-bold');
        } else {
            this.els.rollAngle.classList.remove('text-red-600', 'font-bold');
        }
        if (data.slip_rate > 0.8) {
            this.els.slipRate.classList.add('text-red-600', 'font-bold');
        } else {
            this.els.slipRate.classList.remove('text-red-600', 'font-bold');
        }
    }

    displaySteeringData(data) {
        this.els.innerWheelAngle.textContent = `${data.inner_wheel_angle.toFixed(2)}°`;
        this.els.outerWheelAngle.textContent = `${data.outer_wheel_angle.toFixed(2)}°`;
        this.els.turningRadius.textContent = `${data.turning_radius.toFixed(2)} m`;
        this.els.wheelSpeedDiff.textContent = `${(data.wheel_speed_diff * 100).toFixed(2)}%`;

        if (this.els.transmissionAngleInner) {
            this.els.transmissionAngleInner.textContent = `${data.transmission_angle_inner.toFixed(1)}°`;
            this.els.transmissionAngleOuter.textContent = `${data.transmission_angle_outer.toFixed(1)}°`;
        }
        if (this.els.linkageInterference) {
            this.els.linkageInterference.textContent = data.linkage_interference ? '⚠️ 是' : '否';
            this.els.linkageInterference.className = data.linkage_interference
                ? 'text-red-600 font-bold' : 'text-green-600';
        }
        if (this.els.deadPointRisk) {
            this.els.deadPointRisk.textContent = data.dead_point_risk ? '⚠️ 是' : '否';
            this.els.deadPointRisk.className = data.dead_point_risk
                ? 'text-yellow-600 font-bold' : 'text-green-600';
        }

        this.drawLinkageDiagram(
            data.pole_angle_input || 0,
            data.inner_wheel_angle,
            data.outer_wheel_angle
        );
    }

    displayStabilityData(data) {
        this.els.yawRate.textContent = `${data.yaw_rate.toFixed(2)}°/s`;
        this.els.rollCenter.textContent = `${data.roll_center_height.toFixed(3)} m`;
        this.els.rolloverRisk.textContent = `${data.rollover_risk.toFixed(1)}%`;
        this.els.lateralAccel.textContent = `${data.lateral_acceleration.toFixed(2)}g`;
        this.els.stabilityIndex.textContent = data.stability_index.toFixed(3);

        if (this.els.effectiveCgHeight) {
            this.els.effectiveCgHeight.textContent = `${data.effective_cg_height.toFixed(3)} m`;
            this.els.effectiveCgLateral.textContent = `${(data.effective_cg_lateral * 100).toFixed(2)} cm`;
            this.els.effectiveYawInertia.textContent = `${data.effective_yaw_inertia.toFixed(0)} kg·m²`;
            this.els.cargoShiftLateral.textContent = `${(data.cargo_shift_lateral * 100).toFixed(3)} cm`;
        }

        const riskEl = this.els.rolloverRisk;
        if (data.rollover_risk > 70) {
            riskEl.className = 'text-2xl font-bold text-red-600';
        } else if (data.rollover_risk > 40) {
            riskEl.className = 'text-2xl font-bold text-yellow-600';
        } else {
            riskEl.className = 'text-2xl font-bold text-green-600';
        }
    }

    addAlert(alert) {
        this.alerts.unshift(alert);
        if (this.alerts.length > 20) this.alerts.pop();
        this.renderAlerts();

        const badge = document.getElementById('alertBadge');
        if (badge) {
            badge.textContent = this.alerts.filter(a => !a.acknowledged).length;
        }
    }

    renderAlerts() {
        if (!this.els.alertList) return;

        this.els.alertList.innerHTML = this.alerts.map(alert => {
            const severityClass = {
                'critical': 'bg-red-100 border-red-300 text-red-800',
                'warning': 'bg-yellow-100 border-yellow-300 text-yellow-800',
                'info': 'bg-blue-100 border-blue-300 text-blue-800'
            }[alert.severity] || 'bg-gray-100';

            const time = new Date(alert.timestamp * 1000).toLocaleTimeString();

            return `
                <div class="p-3 rounded border ${severityClass} ${alert.acknowledged ? 'opacity-50' : ''}">
                    <div class="flex justify-between items-center">
                        <span class="font-bold uppercase text-xs">${alert.severity} - ${alert.alert_type}</span>
                        <span class="text-xs">${time}</span>
                    </div>
                    <div class="text-sm mt-1">${alert.message}</div>
                    <div class="text-xs mt-1">车辆: ${alert.vehicle_id} | 值: ${alert.value.toFixed(2)} | 阈值: ${alert.threshold}</div>
                </div>
            `;
        }).join('');
    }

    drawLinkageDiagram(poleAngleDeg, innerAngleDeg, outerAngleDeg) {
        const canvas = document.getElementById('linkageCanvas');
        if (!canvas) return;
        const ctx = canvas.getContext('2d');

        const W = canvas.width;
        const H = canvas.height;
        const cx = W / 2;
        const cy = H * 0.55;
        const scale = 120;

        ctx.clearRect(0, 0, W, H);

        const L = 2.5 * scale;
        const T = 1.8 * scale;

        const frontY = cy - L * 0.3;
        const rearY = cy + L * 0.7;
        const leftX = cx - T / 2;
        const rightX = cx + T / 2;

        ctx.strokeStyle = '#4a5568';
        ctx.lineWidth = 2;
        ctx.strokeRect(leftX - 10, frontY - 10, T + 20, L + 20);

        this._drawWheel(ctx, leftX, frontY, outerAngleDeg, '#e53e3e');
        this._drawWheel(ctx, rightX, frontY, innerAngleDeg, '#38a169');
        this._drawWheel(ctx, leftX, rearY, 0, '#718096');
        this._drawWheel(ctx, rightX, rearY, 0, '#718096');

        const poleLen = 1.8 * scale;
        const poleRad = (poleAngleDeg * Math.PI) / 180;
        const poleEndX = cx + poleLen * Math.sin(poleRad);
        const poleEndY = frontY - 20 - poleLen * Math.cos(poleRad);

        ctx.strokeStyle = '#8b4513';
        ctx.lineWidth = 6;
        ctx.beginPath();
        ctx.moveTo(cx, frontY - 20);
        ctx.lineTo(poleEndX, poleEndY);
        ctx.stroke();

        ctx.fillStyle = '#b87333';
        ctx.beginPath();
        ctx.arc(poleEndX, poleEndY, 8, 0, Math.PI * 2);
        ctx.fill();

        const armLen = 0.25 * scale;
        const ackRad = 12 * Math.PI / 180;

        const outerRad = outerAngleDeg * Math.PI / 180;
        const innerRad = innerAngleDeg * Math.PI / 180;

        const leftArmAngle = Math.PI / 2 - ackRad + outerRad;
        const leftArmX = leftX + armLen * Math.cos(leftArmAngle);
        const leftArmY = frontY + armLen * Math.sin(leftArmAngle);

        const rightArmAngle = Math.PI / 2 + ackRad + innerRad;
        const rightArmX = rightX + armLen * Math.cos(rightArmAngle);
        const rightArmY = frontY + armLen * Math.sin(rightArmAngle);

        ctx.strokeStyle = '#4a5568';
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(leftX, frontY);
        ctx.lineTo(leftArmX, leftArmY);
        ctx.moveTo(rightX, frontY);
        ctx.lineTo(rightArmX, rightArmY);
        ctx.stroke();

        ctx.strokeStyle = '#38a169';
        ctx.lineWidth = 3;
        ctx.setLineDash([5, 5]);
        ctx.beginPath();
        ctx.moveTo(leftArmX, leftArmY);
        ctx.lineTo(rightArmX, rightArmY);
        ctx.stroke();
        ctx.setLineDash([]);

        ctx.fillStyle = '#38a169';
        ctx.beginPath();
        ctx.arc(leftArmX, leftArmY, 5, 0, Math.PI * 2);
        ctx.arc(rightArmX, rightArmY, 5, 0, Math.PI * 2);
        ctx.fill();

        ctx.font = '12px sans-serif';
        ctx.fillStyle = '#1a202c';
        ctx.fillText(`δi=${innerAngleDeg.toFixed(2)}°`, rightX + 10, frontY - 10);
        ctx.fillText(`δo=${outerAngleDeg.toFixed(2)}°`, leftX - 60, frontY - 10);
        ctx.fillText(`辕杆=${poleAngleDeg.toFixed(1)}°`, cx - 30, frontY + 25);

        if (Math.abs(poleAngleDeg) > 0.1) {
            const R = 2.5 / Math.tan(Math.abs(poleAngleDeg) * Math.PI / 180);
            ctx.fillStyle = '#718096';
            ctx.fillText(`R≈${R.toFixed(1)}m`, cx + 80, cy - 80);
        }
    }

    _drawWheel(ctx, x, y, angleDeg, color) {
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(angleDeg * Math.PI / 180);

        ctx.fillStyle = color;
        ctx.fillRect(-15, -8, 30, 16);

        ctx.strokeStyle = '#2d3748';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.ellipse(0, 0, 18, 18, 0, 0, Math.PI * 2);
        ctx.stroke();

        ctx.restore();
    }

    _updateVehicleList(vehicleId) {
        if (!this.els.vehicleSelect) return;
        const exists = Array.from(this.els.vehicleSelect.options).some(o => o.value === vehicleId);
        if (!exists) {
            const opt = document.createElement('option');
            opt.value = vehicleId;
            opt.textContent = vehicleId;
            this.els.vehicleSelect.appendChild(opt);
        }
    }

    _updateVehicleListFromSnapshot(snapshot) {
        if (!this.els.vehicleSelect || !snapshot.vehicles) return;
        snapshot.vehicles.forEach(vid => this._updateVehicleList(vid));
    }

    updateConnectionStatus(connected) {
        const statusEl = document.getElementById('connectionStatus');
        if (statusEl) {
            statusEl.innerHTML = connected
                ? '<span class="w-3 h-3 bg-green-500 rounded-full inline-block mr-2"></span>在线'
                : '<span class="w-3 h-3 bg-red-500 rounded-full inline-block mr-2"></span>离线';
        }
    }

    async runManualAnalysis() {
        const poleAngle = this.manualPoleAngle;
        const speed = this.manualSpeed;
        const cargoMass = parseFloat(this.els.cargoMass.value) || 0;
        const cargoOffsetX = parseFloat(this.els.cargoOffsetX.value) || 0;
        const cargoOffsetY = parseFloat(this.els.cargoOffsetY.value) || 0;
        const cargoOffsetZ = parseFloat(this.els.cargoOffsetZ.value) || 0;

        try {
            const [steeringResp, stabilityResp] = await Promise.all([
                fetch('/api/analysis/steering', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        pole_angle: poleAngle,
                        vehicle_speed: speed,
                        friction_coeff: 0.6
                    })
                }),
                fetch('/api/analysis/stability', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        speed: speed,
                        pole_angle: poleAngle,
                        roll_angle: 10,
                        slip_rate: 0.1,
                        friction_coeff: 0.6,
                        cargo_mass: cargoMass,
                        cargo_offset_lateral: cargoOffsetX,
                        cargo_offset_longitudinal: cargoOffsetY,
                        cargo_offset_height: cargoOffsetZ
                    })
                })
            ]);

            const steering = await steeringResp.json();
            const stability = await stabilityResp.json();

            this.displaySteeringData({ ...steering, pole_angle_input: poleAngle });
            this.displayStabilityData(stability);

            this.chariot3d.setVehicleSpeed(speed);
            this.chariot3d.updateSteering(poleAngle, steering);
            this.chariot3d.updateWheelRotation(
                1 - steering.wheel_speed_diff / 2,
                1 + steering.wheel_speed_diff / 2
            );
            this.chariot3d.setRolloverRisk(stability.rollover_risk);
            this.chariot3d.updateTrajectory(steering, speed);

        } catch (e) {
            console.error('手动分析失败:', e);
        }
    }

    async fetchLatestData() {
        try {
            const resp = await fetch(`/api/data/latest/${this.currentVehicleId}`);
            const data = await resp.json();
            if (data.sensor_data) this.displaySensorData(data.sensor_data);
            if (data.steering_result) this.displaySteeringData(data.steering_result);
            if (data.stability_result) this.displayStabilityData(data.stability_result);
        } catch (e) {
            console.error('获取最新数据失败:', e);
        }
    }

    async fetchHistory() {
        try {
            const resp = await fetch(`/api/data/history?vehicle_id=${this.currentVehicleId}&limit=30`);
            const data = await resp.json();
            this.renderHistory(data);
        } catch (e) {
            console.error('获取历史数据失败:', e);
        }
    }

    renderHistory(data) {
        const historyDiv = document.getElementById('historyContent');
        if (!historyDiv || !data.values) return;

        historyDiv.innerHTML = `
            <h4 class="font-bold mb-2">历史数据 (最近${data.values.length}条)</h4>
            <div class="overflow-x-auto">
                <table class="w-full text-sm">
                    <thead>
                        <tr class="border-b">
                            <th class="text-left p-1">时间</th>
                            <th class="text-left p-1">辕杆(°)</th>
                            <th class="text-left p-1">内轮(°)</th>
                            <th class="text-left p-1">横摆(°/s)</th>
                            <th class="text-left p-1">侧翻(%)</th>
                        </tr>
                    </thead>
                    <tbody>
                        ${data.values.map(row => `
                            <tr class="border-b hover:bg-gray-50">
                                <td class="p-1">${new Date(row[0] * 1000).toLocaleTimeString()}</td>
                                <td class="p-1">${row[1]?.toFixed(1) || '-'}</td>
                                <td class="p-1">${row[2]?.toFixed(2) || '-'}</td>
                                <td class="p-1">${row[3]?.toFixed(2) || '-'}</td>
                                <td class="p-1 ${(row[4] || 0) > 40 ? 'text-yellow-600' : ''} ${(row[4] || 0) > 70 ? 'text-red-600 font-bold' : ''}">${row[4]?.toFixed(1) || '-'}</td>
                            </tr>
                        `).join('')}
                    </tbody>
                </table>
            </div>
        `;
    }

    startAutoRefresh() {
        this.autoRefreshInterval = setInterval(() => {
            if (this.ws && this.ws.readyState === WebSocket.OPEN) {
                this.ws.send(JSON.stringify({ type: 'ping' }));
            }
        }, 30000);
    }

    destroy() {
        if (this.ws) this.ws.close();
        if (this.autoRefreshInterval) clearInterval(this.autoRefreshInterval);
    }
}

window.SteeringPanel = SteeringPanel;
