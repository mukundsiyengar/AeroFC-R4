// plots.js - scrolling strip charts on a canvas (real-time plots of RC input, attitude, heading, altitude, motors)
export class StripChart {
  constructor(canvas, { title, series, min = null, max = null, window = 20, unit = "" }) {
    this.c = canvas;
    this.title = title;
    this.series = series;      // [{ name, color, dash }]
    this.fixed = min !== null && max !== null ? [min, max] : null;
    this.window = window;
    this.unit = unit;
    this.t = [];
    this.data = series.map(() => []);
  }
  push(t, values) {
    this.t.push(t);
    values.forEach((v, i) => this.data[i].push(v));
    while (this.t.length && this.t[0] < t - this.window) { this.t.shift(); this.data.forEach((d) => d.shift()); }
  }
  clear() { this.t = []; this.data = this.series.map(() => []); }

  draw() {
    const dpr = window.devicePixelRatio || 1;
    const r = this.c.getBoundingClientRect();
    const W = Math.round(r.width * dpr), H = Math.round(r.height * dpr);
    if (this.c.width !== W || this.c.height !== H) { this.c.width = W; this.c.height = H; }
    const g = this.c.getContext("2d");
    const css = getComputedStyle(document.documentElement);
    const fg = css.getPropertyValue("--fg").trim() || "#111", muted = css.getPropertyValue("--muted").trim() || "#666";
    const line = css.getPropertyValue("--line").trim() || "#ccc";
    g.clearRect(0, 0, W, H);
    const padL = 40 * dpr, padR = 8 * dpr, padT = 18 * dpr, padB = 14 * dpr;
    const pw = W - padL - padR, ph = H - padT - padB;
    let lo, hi;
    if (this.fixed) [lo, hi] = this.fixed;
    else {
      lo = Infinity; hi = -Infinity;
      for (const d of this.data) for (const v of d) { if (Number.isFinite(v)) { lo = Math.min(lo, v); hi = Math.max(hi, v); } }
      if (!Number.isFinite(lo)) { lo = -1; hi = 1; }
      const span = Math.max(hi - lo, 0.5);
      lo -= span * 0.1; hi += span * 0.1;
    }
    const tEnd = this.t.length ? this.t[this.t.length - 1] : 0, tStart = tEnd - this.window;
    const X = (t) => padL + (t - tStart) / this.window * pw;
    const Y = (v) => padT + (1 - (v - lo) / (hi - lo)) * ph;
    g.strokeStyle = line; g.lineWidth = 1;
    g.font = `${10 * dpr}px ui-monospace, monospace`; g.fillStyle = muted;
    for (let k = 0; k <= 4; k++) {
      const v = lo + (hi - lo) * k / 4, y = Y(v);
      g.beginPath(); g.moveTo(padL, y); g.lineTo(W - padR, y); g.stroke();
      g.fillText(fmt(v), 2 * dpr, y + 3 * dpr);
    }
    this.data.forEach((d, i) => {
      const s = this.series[i];
      g.strokeStyle = s.color; g.lineWidth = 1.6 * dpr; g.setLineDash(s.dash ? [5 * dpr, 4 * dpr] : []);
      g.beginPath();
      let started = false;
      for (let k = 0; k < d.length; k++) {
        if (!Number.isFinite(d[k])) { started = false; continue; }
        const x = X(this.t[k]), y = Y(Math.max(lo, Math.min(hi, d[k])));
        if (!started) { g.moveTo(x, y); started = true; } else g.lineTo(x, y);
      }
      g.stroke();
    });
    g.setLineDash([]);
    g.fillStyle = fg; g.font = `600 ${11 * dpr}px system-ui, sans-serif`;
    g.fillText(this.title + (this.unit ? ` (${this.unit})` : ""), padL, 12 * dpr);
    let x = padL + g.measureText(this.title + (this.unit ? ` (${this.unit})` : "")).width + 12 * dpr;
    g.font = `${10 * dpr}px system-ui, sans-serif`;
    this.series.forEach((s, i) => {
      const last = this.data[i].length ? this.data[i][this.data[i].length - 1] : NaN;
      g.fillStyle = s.color;
      const txt = `${s.name} ${Number.isFinite(last) ? fmt(last) : "-"}`;
      g.fillText(txt, x, 12 * dpr);
      x += g.measureText(txt).width + 10 * dpr;
    });
  }
}
function fmt(v) { return Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2); }
