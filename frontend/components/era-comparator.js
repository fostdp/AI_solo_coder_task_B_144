/**
 * era-comparator.js
 * =================
 * 前端组件 #2：跨时代对比面板
 * 职责：古代车辆 vs 现代汽车 代际进化对比 UI 与 API
 *
 * 对外API：
 *   const era = new EraComparator('#eraResult');
 *   era.run({ancient:['chariot_double'], modern:['modern_car'], ...})
 */
(function (global) {
  'use strict';

  class EraComparator {
    constructor(containerSelector) {
      this.container = document.querySelector(containerSelector);
    }

    async run(request) {
      const resp = await fetch('/api/comparison/eras', {
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
      if (!this.container || !result) return;
      const anc = (result.entries || []).filter(e => e.era === '古代');
      const mod = (result.entries || []).filter(e => e.era === '现代');
      const stat = (arr, k) => arr.length ? (arr.reduce((s, e) => s + (Number(e[k]) || 0), 0) / arr.length) : 0;
      const mkRow = (label, k, unit) => {
        const a = stat(anc, k);
        const m = stat(mod, k);
        const better = (k === 'rollover_risk' || k === 'cg_height' || k === 'ackermann_error' || k === 'turning_radius')
          ? (a < m ? '古代' : '现代')
          : (a > m ? '古代' : '现代');
        return `<tr>
          <td>${label}</td>
          <td style="text-align:right;">${Number.isFinite(a) ? a.toFixed(3) : '-'}${unit || ''}</td>
          <td style="text-align:right;">${Number.isFinite(m) ? m.toFixed(3) : '-'}${unit || ''}</td>
          <td><b>${better}</b></td>
        </tr>`;
      };
      this.container.innerHTML = `
        <h3 style="color:#00d9ff;">${result.title || '跨时代对比'}</h3>
        <p style="color:#aaa;">${result.subtitle || ''}</p>
        <table class="comparison-table" style="margin-top:10px;">
          <thead><tr>
            <th>指标</th><th>古代平均</th><th>现代平均</th><th>领先</th>
          </tr></thead>
          <tbody>
            ${mkRow('不足转向梯度 K_us', 'understeer_gradient', ' °/g')}
            ${mkRow('侧翻风险', 'rollover_risk', ' %')}
            ${mkRow('转弯半径', 'turning_radius', ' m')}
            ${mkRow('稳定性指数', 'stability_index', '')}
            ${mkRow('SSF 静态稳定系数', 'ssf_static', '')}
            ${mkRow('阿克曼误差', 'ackermann_error', ' %')}
            ${mkRow('最高车速', 'max_speed_mps', ' m/s')}
            ${mkRow('重心高', 'cg_height', ' m')}
          </tbody>
        </table>
        ${(result.insights || []).length ? `
          <h4 style="margin-top:15px;color:#ffd347;">📜 研究洞察</h4>
          <ul class="era-insights" style="padding-left:18px;">
            ${result.insights.map(i => `<li style="margin:6px 0;line-height:1.6;">💡 ${i}</li>`).join('')}
          </ul>` : ''}
      `;
    }
  }

  global.EraComparator = EraComparator;
})(window);
