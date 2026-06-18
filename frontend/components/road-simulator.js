/**
 * road-simulator.js
 * =================
 * 前端组件 #3：路面影响仿真面板
 * 职责：多路面对比可视化、摩擦系数/侧翻风险/安全车速对比
 *
 * 对外API：
 *   const road = new RoadSimulator('#roadComparisonTable', '#roadWinners', '#roadInsights');
 *   road.run({vehicle_type:'chariot_double', pole_angle_deg:20, speed_mps:5, roads:[...]});
 */
(function (global) {
  'use strict';

  const COLUMNS = [
    'road_name', 'category', 'friction_coeff', 'rolling_resistance',
    'slip_factor', 'turning_radius_effective', 'yaw_rate',
    'lateral_acceleration', 'rollover_risk', 'stability_index',
    'critical_speed', 'max_safe_speed', 'traction_force_required', 'vibration_level'
  ];
  const LOWER_BETTER = {
    'rollover_risk': true, 'vibration_level': true, 'rolling_resistance': true,
    'turning_radius_effective': true, 'traction_force_required': true, 'slip_factor': true
  };
  const HIGHER_BETTER = {
    'friction_coeff': true, 'stability_index': true, 'critical_speed': true,
    'max_safe_speed': true
  };
  const LABELS = {
    'road_name': '路面类型', 'category': '类别',
    'friction_coeff': '摩擦系数 μ', 'rolling_resistance': '滚动阻力系数',
    'slip_factor': '滑移因子', 'turning_radius_effective': '有效转弯半径(m)',
    'yaw_rate': '横摆率(°/s)', 'lateral_acceleration': '侧向加速度(g)',
    'rollover_risk': '侧翻风险(%)', 'stability_index': '稳定性指数',
    'critical_speed': '临界车速(m/s)', 'max_safe_speed': '安全车速(m/s)',
    'traction_force_required': '牵引力需求(N)', 'vibration_level': '振动等级(m/s²)'
  };

  function _renderTable(tableEl, entries, cols, lowerBetter, higherBetter, labels) {
    if (!tableEl || !entries || !entries.length) return;
    const best = {};
    cols.forEach(c => {
      if (lowerBetter[c]) best[c] = Math.min(...entries.map(e => typeof e[c] === 'number' ? e[c] : Infinity));
      if (higherBetter[c]) best[c] = Math.max(...entries.map(e => typeof e[c] === 'number' ? e[c] : -Infinity));
    });
    const thead = tableEl.querySelector('thead');
    if (thead) thead.innerHTML = '<tr>' + cols.map(c => `<th>${labels[c] || c}</th>`).join('') + '</tr>';
    const tbody = tableEl.querySelector('tbody');
    if (!tbody) return;
    tbody.innerHTML = entries.map(e => '<tr>' + cols.map(c => {
      let v = e[c];
      let cls = '';
      if (typeof v === 'number') {
        if (v === best[c]) cls = 'best-value';
        v = Math.abs(v) > 100 ? v.toFixed(0) : v.toFixed(3);
      }
      return `<td class="${cls}">${v ?? '-'}</td>`;
    }).join('') + '</tr>').join('');
  }

  class RoadSimulator {
    constructor(tableSelector, winnersSelector, insightsSelector, summarySelector) {
      this.tableEl = document.querySelector(tableSelector);
      this.winnersEl = winnersSelector ? document.querySelector(winnersSelector) : null;
      this.insightsEl = insightsSelector ? document.querySelector(insightsSelector) : null;
      this.summaryEl = summarySelector ? document.querySelector(summarySelector) : null;
    }

    async run(request) {
      const resp = await fetch('/api/comparison/roads', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(request || {})
      });
      if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
      const result = await resp.json();
      this.render(result);
      return result;
    }

    render(result) {
      if (!result) return;
      if (this.summaryEl) {
        this.summaryEl.innerHTML = `
          <h4 style="color:#00d9ff;margin:0;">${result.title}</h4>
          <div style="font-size:12px;color:#aaa;margin-top:4px;">${result.subtitle}</div>
        `;
      }
      if (this.winnersEl && result.winners) {
        this.winnersEl.innerHTML = Object.entries(result.winners).map(([k, v]) => `
          <div class="winner-card">
            <div class="metric-name">${k}</div>
            <div class="metric-winner">🏆 ${v}</div>
          </div>
        `).join('');
      }
      if (this.insightsEl && result.insights) {
        this.insightsEl.innerHTML = result.insights.map(i => {
          let cls = '';
          if (i.includes('⚠') || i.includes('危险')) cls = 'danger';
          if (i.includes('风险')) cls = 'warning';
          return `<li class="${cls}">🛣️ ${i}</li>`;
        }).join('');
      }
      _renderTable(this.tableEl, result.entries, COLUMNS, LOWER_BETTER, HIGHER_BETTER, LABELS);
    }

    renderRoadList(roadTypes) {
      if (!this.tableEl) return;
    }
  }

  global.RoadSimulator = RoadSimulator;
  global.RoadSimulatorRenderTable = _renderTable;
})(window);
