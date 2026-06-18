/**
 * vr-chariot.js
 * =============
 * 前端组件 #4：虚拟驾驶双辕车体验
 * 职责：虚拟驾驶 UI 控制、键盘/鼠标辕杆输入、状态渲染、力反馈可视化
 *
 * 对外API：
 *   const vr = new VRChariotEngine({
 *     canvasId: 'vrCanvas',
 *     statusPanel: '#vrStatus',
 *     ffbMeter: '#ffbMeter',
 *     poleSliderId: 'poleSlider'
 *   });
 *   vr.start();   // 开始循环
 *   vr.stop();    // 停止循环
 */
(function (global) {
  'use strict';

  const KEY_MAP = {
    'KeyW': 'throttle_up', 'ArrowUp: 'throttle_up',
    'KeyS': 'brake', 'ArrowDown': 'brake',
    'KeyA': 'pole_left', 'ArrowLeft': 'pole_left',
    'KeyD': 'pole_right', 'ArrowRight': 'pole_right',
    'Space': 'reset'
  };

  class VRChariotEngine {
    constructor(options) {
      options = options || {};
      this.canvasId = options.canvasId || 'vrCanvas';
      this.statusSelector = options.statusPanel || '#vrStatus';
      this.ffbSelector = options.ffbMeter || '#ffbMeter';
      this.poleSliderId = options.poleSliderId || 'poleSlider';
      this.sessionId = 'vr-sess-' + Math.random().toString(36).slice(2, 10);

      this.state = null;
      this.keys = {};
      this.running = false;
      this.animationId = null;
      this.lastTime = 0;
      this.throttle = 0;
      this.brake = 0;
      this.poleAngle = 0;
      this.targetPoleAngle = 0;

      this.vehicleType = options.vehicleType || 'chariot_double';
      this.roadType = options.roadType || 'ancient_post_road';

      this.onStateChange = options.onStateChange || null;
      this.onFfbUpdate = options.onFfbUpdate || null;

      this._bindEvents();
    }

    _bindEvents() {
      document.addEventListener('keydown', e => {
        const action = KEY_MAP[e.code];
        if (!action) return;
        e.preventDefault();
        if (action === 'reset') { this.resetSession(); return; }
        this.keys[action] = true;
      });
      document.addEventListener('keyup', e => {
        const action = KEY_MAP[e.code];
        if (!action) return;
        this.keys[action] = false;
      });

      const slider = document.getElementById(this.poleSliderId);
      if (slider) {
        slider.addEventListener('input', e => {
          this.targetPoleAngle = parseFloat(e.target.value);
        });
      }

      const canvas = document.getElementById(this.canvasId);
      if (canvas) {
        let dragging = false;
        const onMove = (clientX) => {
          if (!dragging) return;
          const rect = canvas.getBoundingClientRect();
          const cx = (clientX - rect.left) / rect.width;
          this.targetPoleAngle = (cx - 0.5) * 80;
          if (slider) slider.value = this.targetPoleAngle;
        };
        canvas.addEventListener('mousedown', () => dragging = true);
        canvas.addEventListener('mouseup', () => dragging = false);
        canvas.addEventListener('mousemove', e => onMove(e.clientX));
        canvas.addEventListener('touchstart', e => { dragging = true; onMove(e.touches[0].clientX); });
        canvas.addEventListener('touchend', () => dragging = false);
        canvas.addEventListener('touchmove', e => { onMove(e.touches[0].clientX); });
      }
    }

    start() {
      if (this.running) return;
      this.running = true;
      this.lastTime = performance.now();
      this._loop();
    }

    stop() {
      this.running = false;
      if (this.animationId) cancelAnimationFrame(this.animationId);
    }

    setVehicle(type) { this.vehicleType = type; this.resetSession(); }
    setRoad(type) { this.roadType = type; this.resetSession(); }

    async resetSession() {
      try {
        await fetch(`/api/virtual-drive/reset/${this.sessionId}`, { method: 'POST' });
      } catch (e) { console.warn('reset failed', e); }
    }

    async _step() {
      if (this.keys.throttle_up) this.throttle = Math.min(1, this.throttle + 0.05);
      else this.throttle = Math.max(0, this.throttle - 0.02);
      if (this.keys.brake) this.brake = Math.min(1, this.brake + 0.1);
      else this.brake = Math.max(0, this.brake - 0.05);

      if (this.keys.pole_left) this.targetPoleAngle = Math.max(-40, this.targetPoleAngle - 2);
      if (this.keys.pole_right) this.targetPoleAngle = Math.min(40, this.targetPoleAngle + 2);
      this.poleAngle += (this.targetPoleAngle - this.poleAngle) * 0.2;

      try {
        const resp = await fetch('/api/virtual-drive/step', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({
            session_id: this.sessionId,
            vehicle_type: this.vehicleType,
            road_type: this.roadType,
            pole_angle_deg: this.poleAngle,
            throttle: this.throttle,
            brake: this.brake,
            dt: 0.05
          })
        });
        if (!resp.ok) return;
        this.state = await resp.json();
        this._renderStatus();
        this._renderFfb();
        if (this.onStateChange) this.onStateChange(this.state);
      } catch (e) { /* ignore */ }
    }

    _loop() {
      if (!this.running) return;
      const now = performance.now();
      const dt = now - this.lastTime;
      if (dt >= 50) {
        this._step();
        this.lastTime = now;
      }
      this._drawCanvas();
      this.animationId = requestAnimationFrame(() => this._loop());
    }

    _drawCanvas() {
      const canvas = document.getElementById(this.canvasId);
      if (!canvas) return;
      const ctx = canvas.getContext('2d');
      const w = canvas.width, h = canvas.height;
      ctx.clearRect(0, 0, w, h);
      // 背景
      const grd = ctx.createRadialGradient(w / 2, h / 2, 10, w / 2, h / 2, w * 0.6);
      grd.addColorStop(0, '#1a2a3a');
      grd.addColorStop(1, '#0a1520');
      ctx.fillStyle = grd;
      ctx.fillRect(0, 0, w, h);
      // 道路
      ctx.fillStyle = '#2a3a2a';
      ctx.fillRect(0, h * 0.4, w, h * 0.2);
      // 车 - 简单俯视
      const cx = w / 2, cy = h / 2;
      ctx.save();
      ctx.translate(cx, cy);
      const heading = this.state ? this.state.heading : 0;
      ctx.rotate(heading);
      // 车身
      ctx.fillStyle = '#c99a5a';
      ctx.fillRect(-40, -25, 80, 50);
      ctx.strokeStyle = '#8b6914';
      ctx.lineWidth = 2;
      ctx.strokeRect(-40, -25, 80, 50);
      // 车轮
      ctx.fillStyle = '#333';
      [[-30, -30], [-30, 20]].forEach(([x, y]) => {
        ctx.fillRect(x, y, 20, 10);
      });
      [[10, -30], [10, 20]].forEach(([x, y]) => {
        ctx.fillRect(x, y, 20, 10);
      });
      // 辕杆
      const poleDeg = this.state ? this.state.pole_angle : 0;
      ctx.strokeStyle = '#8b5a2b';
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.moveTo(-40, 0);
      const poleLen = 60;
      const dx = poleLen * Math.cos(poleDeg * Math.PI / 180);
      const dy = poleLen * Math.sin(poleDeg * Math.PI / 180);
      ctx.lineTo(-40 - dx, dy);
      ctx.stroke();
      ctx.restore();
      // 速度表
      ctx.fillStyle = '#00ff88';
      ctx.font = '14px monospace';
      const speed = this.state ? this.state.speed : 0;
      ctx.fillText(`速度: ${(speed * 3.6).toFixed(1)} km/h`, 10, 20);
      ctx.fillText(`辕杆角: ${(this.poleAngle || 0).toFixed(1)}°`, 10, 40);
      if (this.state) {
        ctx.fillStyle = this.state.rollover_risk > 70 ? '#ff4444' : '#00ff88';
        ctx.fillText(`侧翻风险: ${this.state.rollover_risk.toFixed(0)}%`, 10, 60);
      }
    }

    _renderStatus() {
      const el = document.querySelector(this.statusSelector);
      if (!el || !this.state) return;
      el.innerHTML = `
        <div class="vr-status-row"><span>速度</span><b>${(this.state.speed * 3.6).toFixed(1)} km/h</b></div>
        <div class="vr-status-row"><span>航向</span><b>${(this.state.heading * 180 / Math.PI).toFixed(1)}°</b></div>
        <div class="vr-status-row"><span>侧翻风险</span><b class="${this.state.rollover_risk > 70 ? 'danger' : ''}">${this.state.rollover_risk.toFixed(0)}%</b></div>
        <div class="vr-status-row"><span>稳定性</span><b>${this.state.stability_index.toFixed(2)}</b></div>
        <div class="vr-status-row"><span>侧偏角</span><b>${this.state.lateral_acceleration.toFixed(2)} g</b></div>
        ${this.state.alert_message ? `<div class="vr-alert">${this.state.alert_message}</div>` : ''}
      `;
    }

    _renderFfb() {
      const el = document.querySelector(this.ffbSelector);
      if (!el || !this.state) return;
      const tot = this.state.ffb_total_torque || 0;
      const intensity = this.state.ffb_intensity || 0;
      const pct = Math.min(100, intensity * 100);
      const color = intensity > 0.7 ? '#ff4444' : intensity > 0.4 ? '#ffaa00' : '#00ff88';
      el.innerHTML = `
        <div style="margin-bottom:6px;">
          <span style="color:#aaa;">力反馈强度</span>
          <b style="float:right;color:${color};">${tot.toFixed(2)} Nm</b>
        </div>
        <div style="height:8px;background:#1a2a3a;border-radius:4px;overflow:hidden;">
          <div style="height:100%;width:${pct}%;background:${color};transition:width 0.1s;"></div>
        </div>
        <div style="font-size:11px;color:#888;margin-top:4px;display:flex;justify-content:space-between;">
          <span>回正: ${(this.state.ffb_aligning_torque || 0).toFixed(2)}</span>
          <span>路感: ${(this.state.ffb_road_feel_torque || 0).toFixed(2)}</span>
          <span>阻尼: ${(this.state.ffb_damping_torque || 0).toFixed(2)}</span>
        </div>
      `;
      if (this.onFfbUpdate) this.onFfbUpdate(this.state);
    }
  }

  global.VRChariotEngine = VRChariotEngine;
})(window);
