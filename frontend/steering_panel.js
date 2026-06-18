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
        this._bindFeatureElements();
        this._bindFeatureEvents();
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

    _bindFeatureElements() {
        const ve = {};
        document.querySelectorAll('.feature-tab').forEach(t => {
            ve['tab_' + t.dataset.tab] = t;
        });
        ['vehicleTypeSelect', 'roadSurfaceSelect',
         'cmpRoadSurface', 'rcmpVehicleType',
         'vDriveVehicleType', 'vDriveRoadSurface',
         'vDrivePoleSlider', 'vDriveReset',
         'btnAccel', 'btnBrake', 'poleWheel', 'poleWheelValue',
         'btnRunVehicleComparison', 'btnRunRoadComparison',
         'winnersGrid', 'roadWinnersGrid',
         'vehicleInsights', 'roadInsights',
         'comparisonSummary', 'vDriveAlert', 'throttleFill'
        ].forEach(id => {
            ve[id] = document.getElementById(id);
        });
        ['vdPosX','vdPosY','vdHeading','vdSpeed','vdRollover','vdStability',
         'vdInnerAngle','vdOuterAngle','vdTurnRadius','vdYawRate',
         'vdLatAccel','vdRoll','vdSlip','vdCargoShift'
        ].forEach(id => {
            ve[id] = document.getElementById(id);
        });
        this.els = { ...this.els, ...ve };

        this._vehicleTypes = [];
        this._roadSurfaces = [];

        this.vDrive = {
            sessionId: null,
            vehicleType: 'chariot_double',
            roadType: 'ancient_post_road',
            poleAngle: 0,
            throttle: 0,
            brake: 0,
            keys: {},
            rafId: null,
            lastStepTime: 0,
            isDragging: false,
            dragStartX: 0,
            dragStartAngle: 0
        };

        this.vDrive3d = null;
    }

    _bindFeatureEvents() {
        document.querySelectorAll('.feature-tab').forEach(tab => {
            tab.addEventListener('click', () => {
                this.switchTab(tab.dataset.tab);
            });
        });

        if (this.els.vehicleTypeSelect) {
            this.els.vehicleTypeSelect.addEventListener('change', (e) => {
                if (this.chariot3d && this.chariot3d.switchVehicleType) {
                    this.chariot3d.switchVehicleType(e.target.value);
                }
            });
        }

        if (this.els.btnRunVehicleComparison) {
            this.els.btnRunVehicleComparison.addEventListener('click', () => this.runVehicleComparison());
        }
        if (this.els.btnRunRoadComparison) {
            this.els.btnRunRoadComparison.addEventListener('click', () => this.runRoadComparison());
        }

        if (this.els.vDriveReset) {
            this.els.vDriveReset.addEventListener('click', () => this.resetVDrive());
        }
        if (this.els.vDrivePoleSlider) {
            this.els.vDrivePoleSlider.addEventListener('input', (e) => {
                this.vDrive.poleAngle = parseFloat(e.target.value);
                this.updatePoleWheel();
            });
        }
        if (this.els.vDriveVehicleType) {
            this.els.vDriveVehicleType.addEventListener('change', (e) => {
                this.vDrive.vehicleType = e.target.value;
                if (this.vDrive3d && this.vDrive3d.switchVehicleType) {
                    this.vDrive3d.switchVehicleType(e.target.value);
                }
                this.resetVDrive();
            });
        }
        if (this.els.vDriveRoadSurface) {
            this.els.vDriveRoadSurface.addEventListener('change', (e) => {
                this.vDrive.roadType = e.target.value;
                this.resetVDrive();
            });
        }
        if (this.els.btnAccel) {
            this.els.btnAccel.addEventListener('mousedown', () => this.vDrive.throttle = 1);
            this.els.btnAccel.addEventListener('mouseup', () => this.vDrive.throttle = 0);
            this.els.btnAccel.addEventListener('mouseleave', () => this.vDrive.throttle = 0);
            this.els.btnAccel.addEventListener('touchstart', (e) => { e.preventDefault(); this.vDrive.throttle = 1; });
            this.els.btnAccel.addEventListener('touchend', () => this.vDrive.throttle = 0);
        }
        if (this.els.btnBrake) {
            this.els.btnBrake.addEventListener('mousedown', () => this.vDrive.brake = 1);
            this.els.btnBrake.addEventListener('mouseup', () => this.vDrive.brake = 0);
            this.els.btnBrake.addEventListener('mouseleave', () => this.vDrive.brake = 0);
            this.els.btnBrake.addEventListener('touchstart', (e) => { e.preventDefault(); this.vDrive.brake = 1; });
            this.els.btnBrake.addEventListener('touchend', () => this.vDrive.brake = 0);
        }

        window.addEventListener('keydown', (e) => this._onKeyDown(e));
        window.addEventListener('keyup', (e) => this._onKeyUp(e));
    }

    _onKeyDown(e) {
        this.vDrive.keys[e.key.toLowerCase()] = true;
        if (['w', 'arrowup'].includes(e.key.toLowerCase())) this.vDrive.throttle = 1;
        if (['s', 'arrowdown', ' '].includes(e.key.toLowerCase())) this.vDrive.brake = 1;
        if (['a', 'arrowleft'].includes(e.key.toLowerCase())) {
            this.vDrive.poleAngle = Math.max(-45, this.vDrive.poleAngle - 2);
            this.els.vDrivePoleSlider && (this.els.vDrivePoleSlider.value = this.vDrive.poleAngle);
            this.updatePoleWheel();
        }
        if (['d', 'arrowright'].includes(e.key.toLowerCase())) {
            this.vDrive.poleAngle = Math.min(45, this.vDrive.poleAngle + 2);
            this.els.vDrivePoleSlider && (this.els.vDrivePoleSlider.value = this.vDrive.poleAngle);
            this.updatePoleWheel();
        }
    }

    _onKeyUp(e) {
        this.vDrive.keys[e.key.toLowerCase()] = false;
        if (['w', 'arrowup'].includes(e.key.toLowerCase()) && !this.vDrive.keys['w'] && !this.vDrive.keys['arrowup']) this.vDrive.throttle = 0;
        if (['s', 'arrowdown', ' '].includes(e.key.toLowerCase())) {
            if (!this.vDrive.keys['s'] && !this.vDrive.keys['arrowdown'] && !this.vDrive.keys[' ']) this.vDrive.brake = 0;
        }
    }

    switchTab(tabId) {
        document.querySelectorAll('.feature-tab').forEach(t => {
            t.classList.toggle('active', t.dataset.tab === tabId);
        });
        document.querySelectorAll('.tab-content').forEach(c => {
            c.classList.toggle('hidden', c.id !== 'tab-' + tabId);
        });

        if (tabId === 'virtual-drive') {
            this.initVirtualDrive3D();
            this.startVDriveLoop();
        } else {
            this.stopVDriveLoop();
        }

        if (tabId === 'comparison') {
            this._ensureComparisonInited();
        }
        if (tabId === 'road-comparison') {
            this._ensureRoadComparisonInited();
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

        if (params.vehicle_types) {
            this._vehicleTypes = params.vehicle_types;
            this._populateVehicleSelects();
        }
        if (params.road_surfaces) {
            this._roadSurfaces = params.road_surfaces;
            this._populateRoadSelects();
        }
    }

    _populateVehicleSelects() {
        const selects = ['vehicleTypeSelect', 'rcmpVehicleType', 'vDriveVehicleType'];
        selects.forEach(sid => {
            const el = document.getElementById(sid);
            if (!el) return;
            const prev = el.value;
            el.innerHTML = this._vehicleTypes.map(v =>
                `<option value="${v.id}" ${v.id === 'chariot_double' ? 'selected' : ''}>${v.name}</option>`
            ).join('');
            if (prev && this._vehicleTypes.some(v => v.id === prev)) el.value = prev;
        });
    }

    _populateRoadSelects() {
        const selects = ['roadSurfaceSelect', 'cmpRoadSurface', 'vDriveRoadSurface'];
        selects.forEach(sid => {
            const el = document.getElementById(sid);
            if (!el) return;
            const prev = el.value;
            el.innerHTML = this._roadSurfaces.map(r =>
                `<option value="${r.id}" ${r.id === 'ancient_post_road' ? 'selected' : ''}>${r.name}</option>`
            ).join('');
            if (prev && this._roadSurfaces.some(r => r.id === prev)) el.value = prev;
        });
    }

    _ensureComparisonInited() {
        const el = document.getElementById('cmpRoadSurface');
        if (el && el.options.length === 0 && this._roadSurfaces.length > 0) {
            this._populateRoadSelects();
        }
    }

    _ensureRoadComparisonInited() {
        const el = document.getElementById('rcmpVehicleType');
        if (el && el.options.length === 0 && this._vehicleTypes.length > 0) {
            this._populateVehicleSelects();
        }
    }

    async runVehicleComparison() {
        const btn = this.els.btnRunVehicleComparison;
        const originalText = btn.textContent;
        btn.textContent = '计算中...';
        btn.disabled = true;

        try {
            const req = {
                pole_angle_deg: parseFloat(document.getElementById('cmpPoleAngle').value) || 15,
                speed_mps: parseFloat(document.getElementById('cmpSpeed').value) || 5,
                road_type: document.getElementById('cmpRoadSurface').value,
                cargo_mass: parseFloat(document.getElementById('cmpCargoMass').value) || 0
            };
            const resp = await fetch('/api/comparison/vehicles', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(req)
            });
            const result = await resp.json();
            this.renderVehicleComparison(result);
        } catch (e) {
            console.error('车辆对比失败:', e);
            alert('对比分析失败: ' + e.message);
        } finally {
            btn.textContent = originalText;
            btn.disabled = false;
        }
    }

    renderVehicleComparison(result) {
        document.getElementById('comparisonSummary').innerHTML = `
            <h4 style="color:#00d9ff;margin-bottom:5px;">${result.title}</h4>
            <div style="font-size:13px;color:#aaa;">${result.subtitle}</div>
        `;

        if (this.els.winnersGrid) {
            this.els.winnersGrid.innerHTML = Object.entries(result.winners).map(([k, v]) => `
                <div class="winner-card">
                    <div class="metric-name">${k}</div>
                    <div class="metric-winner">🏆 ${v}</div>
                </div>
            `).join('');
        }

        if (this.els.vehicleInsights) {
            this.els.vehicleInsights.innerHTML = result.insights.map(i => {
                let cls = '';
                if (i.includes('⚠') || i.includes('风险')) cls = 'warning';
                if (i.includes('危险')) cls = 'danger';
                return `<li class="${cls}">💡 ${i}</li>`;
            }).join('');
        }

        this._renderComparisonTable(
            'vehicleComparisonTable',
            result.entries,
            ['vehicle_name', 'era', 'category', 'steering_mechanism',
             'inner_wheel_angle', 'outer_wheel_angle', 'turning_radius',
             'ackermann_error', 'max_inner_wheel_angle', 'min_turning_radius',
             'yaw_rate', 'lateral_acceleration', 'rollover_risk', 'stability_index',
             'critical_speed', 'ssf_static', 'understeer_gradient',
             'max_speed_mps', 'mass', 'cg_height', 'wheelbase', 'track_width', 'propulsion'],
            {
                'turning_radius': true, 'ackermann_error': true, 'rollover_risk': true,
                'understeer_gradient': true, 'min_turning_radius': true, 'cg_height': true
            },
            {
                'stability_index': true, 'critical_speed': true, 'ssf_static': true,
                'max_speed_mps': true, 'max_inner_wheel_angle': true
            },
            {
                'vehicle_name': '车型', 'era': '时代', 'category': '类别',
                'steering_mechanism': '转向机构', 'inner_wheel_angle': '内轮角(°)',
                'outer_wheel_angle': '外轮角(°)', 'turning_radius': '转弯半径(m)',
                'ackermann_error': '阿克曼误差(%)', 'max_inner_wheel_angle': '最大内轮角(°)',
                'min_turning_radius': '最小转弯半径(m)', 'yaw_rate': '横摆率(°/s)',
                'lateral_acceleration': '侧向加速度(g)', 'rollover_risk': '侧翻风险(%)',
                'stability_index': '稳定性指数', 'critical_speed': '临界速度(m/s)',
                'ssf_static': 'SSF静态', 'understeer_gradient': '不足转向(°/g)',
                'max_speed_mps': '最高车速(m/s)', 'mass': '质量(kg)',
                'cg_height': '重心高(m)', 'wheelbase': '轴距(m)',
                'track_width': '轮距(m)', 'propulsion': '动力'
            }
        );
    }

    async runRoadComparison() {
        const btn = this.els.btnRunRoadComparison;
        const originalText = btn.textContent;
        btn.textContent = '计算中...';
        btn.disabled = true;

        try {
            const req = {
                vehicle_type: document.getElementById('rcmpVehicleType').value,
                pole_angle_deg: parseFloat(document.getElementById('rcmpPoleAngle').value) || 15,
                speed_mps: parseFloat(document.getElementById('rcmpSpeed').value) || 5,
                cargo_mass: parseFloat(document.getElementById('rcmpCargoMass').value) || 0
            };
            const resp = await fetch('/api/comparison/roads', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(req)
            });
            const result = await resp.json();
            this.renderRoadComparison(result);
        } catch (e) {
            console.error('路面对比失败:', e);
            alert('对比分析失败: ' + e.message);
        } finally {
            btn.textContent = originalText;
            btn.disabled = false;
        }
    }

    renderRoadComparison(result) {
        if (this.els.roadWinnersGrid) {
            this.els.roadWinnersGrid.innerHTML = Object.entries(result.winners).map(([k, v]) => `
                <div class="winner-card">
                    <div class="metric-name">${k}</div>
                    <div class="metric-winner">🏆 ${v}</div>
                </div>
            `).join('');
        }

        if (this.els.roadInsights) {
            this.els.roadInsights.innerHTML = result.insights.map(i => {
                let cls = '';
                if (i.includes('⚠') || i.includes('危险')) cls = 'danger';
                else if (i.includes('建议') || i.includes('风险')) cls = 'warning';
                return `<li class="${cls}">🛣️ ${i}</li>`;
            }).join('');
        }

        this._renderComparisonTable(
            'roadComparisonTable',
            result.entries,
            ['road_name', 'category', 'friction_coeff', 'rolling_resistance', 'slip_factor',
             'effective_speed', 'turning_radius_effective', 'yaw_rate', 'lateral_acceleration',
             'rollover_risk', 'stability_index', 'critical_speed', 'ackermann_error',
             'max_safe_speed', 'traction_force_required', 'vibration_level'],
            {
                'rolling_resistance': true, 'slip_factor': true, 'rollover_risk': true,
                'turning_radius_effective': true, 'traction_force_required': true,
                'vibration_level': true, 'ackermann_error': true
            },
            {
                'friction_coeff': true, 'stability_index': true, 'max_safe_speed': true,
                'critical_speed': true, 'effective_speed': true
            },
            {
                'road_name': '路面名称', 'category': '类别',
                'friction_coeff': '摩擦系数μ', 'rolling_resistance': '滚动阻力系数',
                'slip_factor': '滑移因子', 'effective_speed': '有效车速(m/s)',
                'turning_radius_effective': '有效转弯半径(m)', 'yaw_rate': '横摆率(°/s)',
                'lateral_acceleration': '侧向加速度(g)', 'rollover_risk': '侧翻风险(%)',
                'stability_index': '稳定性指数', 'critical_speed': '临界速度(m/s)',
                'ackermann_error': '阿克曼误差(%)', 'max_safe_speed': '安全车速(m/s)',
                'traction_force_required': '牵引力需求(N)', 'vibration_level': '颠簸等级'
            }
        );
    }

    _renderComparisonTable(tableId, entries, columns, lowerBetter, higherBetter, colLabels) {
        const table = document.getElementById(tableId);
        if (!table || !entries || entries.length === 0) return;

        const bestVals = {};
        columns.forEach(col => {
            if (lowerBetter[col]) {
                bestVals[col] = Math.min(...entries.map(e => typeof e[col] === 'number' ? e[col] : Infinity));
            } else if (higherBetter[col]) {
                bestVals[col] = Math.max(...entries.map(e => typeof e[col] === 'number' ? e[col] : -Infinity));
            }
        });

        const thead = table.querySelector('thead');
        thead.innerHTML = '<tr>' + columns.map(c =>
            `<th data-col="${c}">${colLabels[c] || c}</th>`
        ).join('') + '</tr>';

        const tbody = table.querySelector('tbody');
        tbody.innerHTML = entries.map(e => '<tr>' + columns.map(c => {
            let v = e[c];
            let cls = '';
            if (typeof v === 'number') {
                if (v === bestVals[c] && (lowerBetter[c] || higherBetter[c])) cls = 'best-value';
                if ((lowerBetter[c] && v === Math.max(...entries.map(x => typeof x[c] === 'number' ? x[c] : -Infinity))) ||
                    (higherBetter[c] && v === Math.min(...entries.map(x => typeof x[c] === 'number' ? x[c] : Infinity)))) {
                    if (v !== bestVals[c]) cls = 'worst-value';
                }
                v = Number.isFinite(v) ? (Math.abs(v) > 100 ? v.toFixed(0) : v.toFixed(2)) : v.toString();
            }
            return `<td class="${cls}">${v ?? '-'}</td>`;
        }).join('') + '</tr>').join('');
    }

    initVirtualDrive3D() {
        if (this.vDrive3d) return;
        this.vDrive3d = new Chariot3D('vDriveCanvas');
        const canvas = document.querySelector('#vDriveCanvas canvas');
        if (canvas) {
            canvas.addEventListener('mousedown', (e) => this._onVDriveDragStart(e));
            canvas.addEventListener('mousemove', (e) => this._onVDriveDragMove(e));
            canvas.addEventListener('mouseup', () => this._onVDriveDragEnd());
            canvas.addEventListener('mouseleave', () => this._onVDriveDragEnd());
            canvas.addEventListener('touchstart', (e) => this._onVDriveDragStart(e.touches[0]));
            canvas.addEventListener('touchmove', (e) => { e.preventDefault(); this._onVDriveDragMove(e.touches[0]); }, { passive: false });
            canvas.addEventListener('touchend', () => this._onVDriveDragEnd());
        }
    }

    _onVDriveDragStart(e) {
        this.vDrive.isDragging = true;
        this.vDrive.dragStartX = e.clientX;
        this.vDrive.dragStartAngle = this.vDrive.poleAngle;
    }

    _onVDriveDragMove(e) {
        if (!this.vDrive.isDragging) return;
        const dx = e.clientX - this.vDrive.dragStartX;
        this.vDrive.poleAngle = Math.max(-45, Math.min(45, this.vDrive.dragStartAngle + dx * 0.3));
        if (this.els.vDrivePoleSlider) this.els.vDrivePoleSlider.value = this.vDrive.poleAngle;
        this.updatePoleWheel();
    }

    _onVDriveDragEnd() {
        this.vDrive.isDragging = false;
    }

    updatePoleWheel() {
        const indicator = document.querySelector('.pole-indicator');
        if (indicator) indicator.style.transform = `translate(-50%, -50%) rotate(${this.vDrive.poleAngle}deg)`;
        if (this.els.poleWheelValue) this.els.poleWheelValue.textContent = `${this.vDrive.poleAngle.toFixed(0)}°`;
    }

    startVDriveLoop() {
        this.stopVDriveLoop();
        this.vDrive.lastStepTime = performance.now();
        const loop = () => {
            const now = performance.now();
            const dt = Math.min(0.1, (now - this.vDrive.lastStepTime) / 1000);
            this.vDrive.lastStepTime = now;
            this.vDriveStep(dt);
            this.vDrive.rafId = requestAnimationFrame(loop);
        };
        this.vDrive.rafId = requestAnimationFrame(loop);
    }

    stopVDriveLoop() {
        if (this.vDrive.rafId) {
            cancelAnimationFrame(this.vDrive.rafId);
            this.vDrive.rafId = null;
        }
    }

    async resetVDrive() {
        if (this.vDrive.sessionId) {
            try { await fetch(`/api/virtual-drive/reset/${this.vDrive.sessionId}`, { method: 'POST' }); } catch (e) {}
            this.vDrive.sessionId = null;
        }
        this.vDrive.poleAngle = 0;
        this.vDrive.throttle = 0;
        this.vDrive.brake = 0;
        if (this.els.vDrivePoleSlider) this.els.vDrivePoleSlider.value = 0;
        this.updatePoleWheel();
    }

    async vDriveStep(dt) {
        if (this.vDrive.throttle === 0 && this.vDrive.brake === 0 &&
            Math.abs(this.vDrive.poleAngle) < 0.1 && !this.vDrive.sessionId) return;

        const cargoMass = parseFloat(document.getElementById('vDriveCargoMass')?.value) || 0;
        const cargoX = parseFloat(document.getElementById('vDriveCargoX')?.value) || 0;

        try {
            const resp = await fetch('/api/virtual-drive/step', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    session_id: this.vDrive.sessionId,
                    vehicle_type: this.vDrive.vehicleType,
                    road_type: this.vDrive.roadType,
                    pole_angle_deg: this.vDrive.poleAngle,
                    throttle: this.vDrive.throttle,
                    brake: this.vDrive.brake,
                    cargo_mass: cargoMass,
                    cargo_offset_lateral: cargoX,
                    dt: dt
                })
            });
            const state = await resp.json();
            this.vDrive.sessionId = state.session_id;
            this.updateVDriveUI(state);

            if (this.vDrive3d) {
                this.vDrive3d.setVehicleSpeed(state.speed);
                this.vDrive3d.updateSteering(state.pole_angle, {
                    inner_wheel_angle: state.inner_wheel_angle,
                    outer_wheel_angle: state.outer_wheel_angle,
                    pole_angle_input: state.pole_angle,
                    wheel_speed_diff: 0
                });
                this.vDrive3d.setRolloverRisk(state.rollover_risk);
                this.vDrive3d.setVehicleHeading(state.heading);
                this.vDrive3d.setVehiclePosition(state.x, state.y);
            }
        } catch (e) {
            console.warn('虚拟驾驶步进失败:', e.message);
        }
    }

    updateVDriveUI(state) {
        const setText = (id, v, suffix = '') => {
            const el = this.els[id];
            if (el && typeof v === 'number') el.textContent = `${Number.isFinite(v) ? v.toFixed(2) : '∞'}${suffix}`;
            else if (el) el.textContent = `${v}${suffix}`;
        };

        setText('vdPosX', state.x, ' m');
        setText('vdPosY', state.y, ' m');
        setText('vdHeading', state.heading, '°');
        setText('vdSpeed', state.speed, ' m/s');
        setText('vdRollover', state.rollover_risk, '%');
        setText('vdStability', state.stability_index);
        setText('vdInnerAngle', state.inner_wheel_angle, '°');
        setText('vdOuterAngle', state.outer_wheel_angle, '°');
        if (this.els.vdTurnRadius) {
            this.els.vdTurnRadius.textContent = (state.turning_radius === Infinity || !Number.isFinite(state.turning_radius))
                ? '∞ m' : `${state.turning_radius.toFixed(2)} m`;
        }
        setText('vdYawRate', state.yaw_rate, '°/s');
        setText('vdLatAccel', state.lateral_acceleration, 'g');
        setText('vdRoll', state.roll_angle, '°');
        setText('vdSlip', (state.slip_ratio || 0) * 100, '%');
        setText('vdCargoShift', (state.cargo_shift_lateral || 0) * 100, ' cm');

        if (this.els.vdRollover) {
            this.els.vdRollover.className = '';
            if (state.rollover_risk > 70) this.els.vdRollover.classList.add('text-red-600', 'font-bold');
            else if (state.rollover_risk > 40) this.els.vdRollover.classList.add('text-yellow-600', 'font-bold');
            else this.els.vdRollover.classList.add('text-green-600');
        }

        if (this.els.vDriveAlert) {
            this.els.vDriveAlert.textContent = state.alert_message || '';
            this.els.vDriveAlert.className = 'drive-alert';
            if (state.alert_message) {
                if (state.is_tipping || state.rollover_risk > 80) this.els.vDriveAlert.classList.add('danger');
                else if (state.rollover_risk > 60 || state.is_stuck) this.els.vDriveAlert.classList.add('warning');
                else this.els.vDriveAlert.classList.add('info');
            }
        }

        if (this.els.throttleFill) {
            const t = this.vDrive.throttle - this.vDrive.brake * 0.5;
            this.els.throttleFill.style.width = `${Math.max(0, Math.min(1, t + 0.5)) * 100}%`;
        }
    }

    destroy() {
        if (this.ws) this.ws.close();
        if (this.autoRefreshInterval) clearInterval(this.autoRefreshInterval);
        this.stopVDriveLoop();
    }
}

window.SteeringPanel = SteeringPanel;
