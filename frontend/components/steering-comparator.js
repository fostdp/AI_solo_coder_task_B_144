/**
 * steering-comparator.js
 * ======================
 * 前端组件 #1：机构对比面板
 * 职责：多车型转向机构对比的UI渲染与后端API调用
 *
 * 独立可运行：
 *   const cmp = new SteeringComparator('#vehicleComparisonTable', '#winnersGrid', '#insightsList');
 *   cmp.run({pole_angle_deg:15, speed_mps:5, road_type:'ancient_post_road'});
 *
 * 对外API：
 *   - constructor(tableSel, winnersSel, insightsSel, summarySel)
 *   - async run(request) => ComparisonResult
 *   - render(result) => void
 */
(function (global) {
  'use strict';

  const DEFAULT_COLUMNS = [
    'vehicle_name', 'era', 'category', 'steering_mechanism',
    'inner_wheel_angle', 'outer_wheel_angle', 'turning_radius',
    'ackermann_error', 'max_inner_wheel_angle', 'min_turning_radius',
    'yaw_rate', 'lateral_acceleration', 'rollover_risk', 'stability_index',
    'critical_speed', 'ssf_static', 'understeer_gradient',
    'max_speed_mps', 'mass', 'cg_height', 'wheelbase', 'track_width', 'propulsion'
  ];
  const LOWER_BETTER = {
    'turning_radius': true, 'ackermann_error': true, 'rollover_risk': true,
    'understeer_gradient': true, 'min_turning_radius': true, 'cg_height': true
  };
  const HIGHER_BETTER = {
    'stability_index': true, 'critical_speed': true, 'ssf_static': true,
    'max_speed_mps': true, 'max_inner_wheel_angle': true
  };
  const COLUMN_LABELS = {
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
  };

  function _renderTable(tableEl, entries, columns, lowerBetter, higherBetter, colLabels) {
    columns = columns || DEFAULT_COLUMNS;
    lowerBetter = lowerBetter || LOWER_BETTER;
    higherBetter = higherBetter || HIGHER_BETTER;
    colLabels = colLabels || COLUMN_LABELS;
    if (!tableEl || !entries || entries.length === 0) return;

    const bestVals = {};
    columns.forEach(col => {
      if (lowerBetter[col]) {
        bestVals[col] = Math.min(...entries.map(e => typeof e[col] === 'number' ? e[col] : Infinity));
      } else if (higherBetter[col]) {
        bestVals[col] = Math.max(...entries.map(e => typeof e[col] === 'number' ? e[col] : -Infinity));
      }
    });

    const thead = tableEl.querySelector('thead');
    if (thead) {
      thead.innerHTML = '<tr>' + columns.map(c =>
        `<th data-col="${c}">${colLabels[c] || c}</th>`
      ).join('') + '</tr>';
    }
    const tbody = tableEl.querySelector('tbody');
    if (!tbody) return;
    tbody.innerHTML = entries.map(e => '<tr>' + columns.map(c => {
      let v = e[c];
      let cls = '';
      if (typeof v === 'number') {
        if (v === bestVals[c] && (lowerBetter[c] || higherBetter[c])) cls = 'best-value';
        const worst = (lowerBetter[c])
          ? Math.max(...entries.map(x => typeof x[c] === 'number' ? x[c] : -Infinity))
          : Math.min(...entries.map(x => typeof x[c] === 'number' ? x[c] : Infinity));
        if (v === worst && v !== bestVals[c]) cls = 'worst-value';
        v = Number.isFinite(v) ? (Math.abs(v) > 100 ? v.toFixed(0) : v.toFixed(2)) : v.toString();
      }
      return `<td class="${cls}">${v ?? '-'}</td>`;
    }).join('') + '</tr>').join('');
  }

  class SteeringComparator {
    constructor(tableSelector, winnersSelector, insightsSelector, summarySelector) {
      this.tableEl = document.querySelector(tableSelector);
      this.winnersEl = winnersSelector ? document.querySelector(winnersSelector) : null;
      this.insightsEl = insightsSelector ? document.querySelector(insightsSelector) : null;
      this.summaryEl = summarySelector ? document.querySelector(summarySelector) : null;
      this.columns = DEFAULT_COLUMNS.slice();
    }

    async run(request) {
      const resp = await fetch('/api/comparison/vehicles', {
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
          <h4 style="color:#00d9ff;margin-bottom:5px;">${result.title}</h4>
          <div style="font-size:13px;color:#aaa;">${result.subtitle}</div>
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
          if (i.includes('⚠') || i.includes('风险')) cls = 'warning';
          if (i.includes('危险')) cls = 'danger';
          return `<li class="${cls}">💡 ${i}</li>`;
        }).join('');
      }
      _renderTable(this.tableEl, result.entries, this.columns, LOWER_BETTER, HIGHER_BETTER, COLUMN_LABELS);
    }
  }

  // 导出到全局
  global.SteeringComparator = SteeringComparator;
  global.SteeringComparatorRenderTable = _renderTable;
})(window);
