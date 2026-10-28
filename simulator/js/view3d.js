// view3d.js - simple perspective view of the drone (no libraries): ground grid, shadow, rotors coloured by thrust.
import { X3, qToMat } from "./physics_x3.js";

export class View3D {
  constructor(canvas) {
    this.c = canvas;
    this.g = canvas.getContext("2d");
    this.camMode = "chase";   // "chase" (behind the drone) or "ground" (fixed observer)
    this.camYaw = 0;
    this.trail = [];
  }

  resize() {
    const r = this.c.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(200, Math.round(r.width * dpr)), h = Math.max(150, Math.round(r.height * dpr));
    if (this.c.width !== w || this.c.height !== h) { this.c.width = w; this.c.height = h; }
  }

  draw(phys, motors, hud, theme) {
    this.resize();
    const g = this.g, W = this.c.width, H = this.c.height;
    const css = getComputedStyle(document.documentElement);
    const col = (n, d) => (css.getPropertyValue(n) || d).trim() || d;
    const sky = col("--sky", "#cfe3f7"), ground = col("--ground", "#dfe7d8"), grid = col("--grid", "#b9c7b0");
    const fg = col("--fg", "#111"), accent = col("--accent", "#2563eb");

    const p = phys.p;
    const R = phys.R || qToMat(phys.q);
    const heading = Math.atan2(R[1][0], R[0][0]);   // body x direction in the world (ENU)
    // camera
    let cam, target;
    if (this.camMode === "chase") {
      const d = this.camYaw - heading;
      this.camYaw -= Math.atan2(Math.sin(d), Math.cos(d)) * 0.05;
      const back = 3.2, up = 1.4;
      cam = [p[0] - back * Math.cos(this.camYaw), p[1] - back * Math.sin(this.camYaw), Math.max(0.3, p[2] + up)];
      target = [p[0], p[1], p[2]];
    } else {
      cam = [0, -6, 1.6];
      target = [p[0], p[1], Math.max(p[2], 0.5)];
    }
    // view basis
    const f = norm(sub(target, cam));
    const r = norm(cross(f, [0, 0, 1]));
    const u = cross(r, f);
    const focal = 0.5 * H / Math.tan(30 * Math.PI / 180);
    const proj = (w) => {
      const v = sub(w, cam);
      const z = dot(v, f);
      if (z < 0.05) return null;
      return [W / 2 + focal * dot(v, r) / z, H / 2 - focal * dot(v, u) / z, z];
    };

    // sky and ground (horizon line)
    const horizon = proj(add(cam, add(scale(f, 1000), [0, 0, -cam[2]])));
    const hy = horizon ? Math.max(0, Math.min(H, horizon[1])) : H / 2;
    g.fillStyle = sky; g.fillRect(0, 0, W, hy);
    g.fillStyle = ground; g.fillRect(0, hy, W, H - hy);

    // grid around the drone (1 m) and the home pad
    g.strokeStyle = grid; g.lineWidth = 1;
    const cx = Math.round(p[0]), cy = Math.round(p[1]), N = 15;
    g.beginPath();
    for (let i = -N; i <= N; i++) {
      line(g, proj, [cx + i, cy - N, 0], [cx + i, cy + N, 0]);
      line(g, proj, [cx - N, cy + i, 0], [cx + N, cy + i, 0]);
    }
    g.stroke();
    g.strokeStyle = accent; g.lineWidth = 2; g.beginPath();
    circle(g, proj, [0, 0, 0], 0.5, 24);
    line(g, proj, [-0.25, -0.25, 0], [-0.25, 0.25, 0]); line(g, proj, [0.25, -0.25, 0], [0.25, 0.25, 0]); line(g, proj, [-0.25, 0, 0], [0.25, 0, 0]);
    g.stroke();
    // north arrow at home
    g.strokeStyle = "#c0392b"; g.beginPath(); line(g, proj, [0, 0.6, 0], [0, 1.4, 0]); line(g, proj, [0, 1.4, 0], [-0.15, 1.15, 0]); line(g, proj, [0, 1.4, 0], [0.15, 1.15, 0]); g.stroke();

    // trail
    if (!this.trail.length || Math.hypot(...sub(p, this.trail[this.trail.length - 1])) > 0.1) {
      this.trail.push(p.slice());
      if (this.trail.length > 300) this.trail.shift();
    }
    g.strokeStyle = accent; g.globalAlpha = 0.45; g.lineWidth = 1.5; g.beginPath();
    for (let i = 1; i < this.trail.length; i++) line(g, proj, this.trail[i - 1], this.trail[i]);
    g.stroke(); g.globalAlpha = 1;

    // shadow + altitude pole
    g.fillStyle = "rgba(0,0,0,0.25)"; g.beginPath(); circle(g, proj, [p[0], p[1], 0.001], 0.3, 20, true); g.fill();
    g.strokeStyle = "rgba(0,0,0,0.35)"; g.setLineDash([4, 4]); g.beginPath(); line(g, proj, [p[0], p[1], 0], p); g.stroke(); g.setLineDash([]);

    // drone: arms, rotor discs coloured by motor command, front arrow
    const toW = (b) => add(p, [R[0][0] * b[0] + R[0][1] * b[1] + R[0][2] * b[2], R[1][0] * b[0] + R[1][1] * b[1] + R[1][2] * b[2], R[2][0] * b[0] + R[2][1] * b[1] + R[2][2] * b[2]]);
    g.lineWidth = 4;
    X3.rotors.forEach((ro, i) => {
      g.strokeStyle = ro.pos[0] > 0 ? "#e67e22" : "#34495e";
      g.beginPath(); line(g, proj, toW([0, 0, 0]), toW(ro.pos)); g.stroke();
    });
    g.lineWidth = 2;
    X3.rotors.forEach((ro, i) => {
      const m = motors ? motors[i] : 0;
      const hue = 120 - 120 * Math.min(1, m / 0.9);
      g.strokeStyle = `hsl(${hue},80%,40%)`; g.fillStyle = `hsla(${hue},80%,50%,0.35)`;
      g.beginPath();
      const pts = [];
      for (let k = 0; k <= 16; k++) { const a = k / 16 * 2 * Math.PI; pts.push(proj(toW([ro.pos[0] + 0.1 * Math.cos(a), ro.pos[1] + 0.1 * Math.sin(a), ro.pos[2]]))); }
      if (pts.every(Boolean)) { pts.forEach((q, k) => (k ? g.lineTo(q[0], q[1]) : g.moveTo(q[0], q[1]))); g.fill(); g.stroke(); }
    });
    g.strokeStyle = "#16a34a"; g.lineWidth = 3; g.beginPath(); line(g, proj, toW([0, 0, 0.06]), toW([0.25, 0, 0.06])); g.stroke();
    g.fillStyle = "#1f2937"; g.beginPath(); circle(g, proj, toW([0, 0, 0.03]), 0.07, 12, true); g.fill();

    // HUD text
    if (hud) {
      g.font = `${Math.round(13 * (window.devicePixelRatio || 1))}px ui-monospace, monospace`;
      g.fillStyle = fg;
      hud.forEach((t, i) => g.fillText(t, 10, 20 + i * 18 * (window.devicePixelRatio || 1)));
    }
  }
}

function line(g, proj, a, b) {
  const p = proj(a), q = proj(b);
  if (!p || !q) return;
  g.moveTo(p[0], p[1]); g.lineTo(q[0], q[1]);
}
function circle(g, proj, c, rad, n, fill = false) {
  let first = true;
  for (let k = 0; k <= n; k++) {
    const a = k / n * 2 * Math.PI;
    const q = proj([c[0] + rad * Math.cos(a), c[1] + rad * Math.sin(a), c[2]]);
    if (!q) { first = true; continue; }
    if (first) { g.moveTo(q[0], q[1]); first = false; } else g.lineTo(q[0], q[1]);
  }
}
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const norm = (a) => { const n = Math.hypot(...a) || 1; return [a[0] / n, a[1] / n, a[2] / n]; };

// artificial horizon (roll / pitch in degrees, FRD convention) + heading
export function drawHorizon(canvas, roll, pitch, heading) {
  const dpr = window.devicePixelRatio || 1;
  const r = canvas.getBoundingClientRect();
  canvas.width = Math.round(r.width * dpr); canvas.height = Math.round(r.height * dpr);
  const g = canvas.getContext("2d"), W = canvas.width, H = canvas.height;
  g.save();
  g.translate(W / 2, H / 2);
  g.rotate(-roll * Math.PI / 180);
  const ppd = H / 60;   // pixels per degree
  g.fillStyle = "#4a90d9"; g.fillRect(-W * 2, -H * 2 + pitch * ppd, W * 4, H * 2);
  g.fillStyle = "#8b6b3e"; g.fillRect(-W * 2, pitch * ppd, W * 4, H * 2);
  g.strokeStyle = "#fff"; g.lineWidth = dpr; g.fillStyle = "#fff"; g.font = `${10 * dpr}px sans-serif`;
  for (let d = -30; d <= 30; d += 10) {
    if (!d) continue;
    const y = (pitch - d) * ppd;
    g.beginPath(); g.moveTo(-W * 0.12, y); g.lineTo(W * 0.12, y); g.stroke();
    g.fillText(String(d), W * 0.14, y + 3 * dpr);
  }
  g.beginPath(); g.moveTo(-W, pitch * ppd); g.lineTo(W, pitch * ppd); g.stroke();
  g.restore();
  g.strokeStyle = "#ffd400"; g.lineWidth = 3 * dpr;
  g.beginPath(); g.moveTo(W * 0.25, H / 2); g.lineTo(W * 0.42, H / 2); g.lineTo(W * 0.46, H / 2 + 8 * dpr);
  g.moveTo(W * 0.75, H / 2); g.lineTo(W * 0.58, H / 2); g.lineTo(W * 0.54, H / 2 + 8 * dpr); g.stroke();
  g.fillStyle = "rgba(0,0,0,0.55)"; g.fillRect(W / 2 - 34 * dpr, 4 * dpr, 68 * dpr, 18 * dpr);
  g.fillStyle = "#fff"; g.font = `${12 * dpr}px ui-monospace, monospace`; g.textAlign = "center";
  g.fillText(`HDG ${Math.round(((heading % 360) + 360) % 360)}`, W / 2, 17 * dpr);
}
