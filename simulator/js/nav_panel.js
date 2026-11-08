// nav_panel.js - the map for flying by itself: seen from above, north up, centred on the take-off point.
// Click to choose where to go, or to add mission points. All positions are metres north / east of the take-off point, the
// same numbers the flight controller gets (CODE_GOTO, CODE_WAYPOINT in firmware/AeroFC_R4/src/core/flight_core.h).
const MIN_RANGE_M = 20;      // the map shows at least this far from home
const TRAIL_POINTS = 600;
export const MAX_WAYPOINTS = 16;

// pixels <-> metres, as plain functions so the tests can check them
export function mapToMetres(px, py, size, rangeM) {
  const k = (2 * rangeM) / size;
  return { north: (size / 2 - py) * k, east: (px - size / 2) * k };
}
export function metresToMap(north, east, size, rangeM) {
  const k = size / (2 * rangeM);
  return { x: size / 2 + east * k, y: size / 2 - north * k };
}
// how far the map must reach to show everything, rounded up to a tidy number, so it doesn't change at every step
export function mapRange(points) {
  let far = MIN_RANGE_M;
  for (const p of points) far = Math.max(far, Math.hypot(p.north, p.east) * 1.15);
  return Math.ceil(far / 10) * 10;
}

export class NavPanel {
  constructor(canvas, onClick) {
    this.c = canvas;
    this.onClick = onClick;
    this.trail = [];
    this.points = [];         // mission points: { north, east, height }
    this.target = null;       // the point clicked last: { north, east }
    this.status = null;
    this.range = MIN_RANGE_M;
    canvas.addEventListener("click", (e) => {
      const r = canvas.getBoundingClientRect();
      onClick(mapToMetres(e.clientX - r.left, e.clientY - r.top, r.width, this.range));
    });
  }

  relative(s) { return { north: s.posN - s.homeN, east: s.posE - s.homeE }; }

  update(s) {
    this.status = s;
    if (s && s.homeN !== undefined && s.armed) {
      const p = this.relative(s);
      const last = this.trail[this.trail.length - 1];
      if (!last || Math.hypot(p.north - last.north, p.east - last.east) > 0.3) {
        this.trail.push(p);
        if (this.trail.length > TRAIL_POINTS) this.trail.shift();
      }
    }
    this.draw();
  }

  draw() {
    const c = this.c, css = getComputedStyle(c), g = c.getContext("2d");
    const size = c.clientWidth || 300, dpr = window.devicePixelRatio || 1;
    if (c.width !== Math.round(size * dpr)) { c.width = Math.round(size * dpr); c.height = Math.round(size * dpr); }
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const s = this.status;
    const drone = s && s.homeN !== undefined ? this.relative(s) : null;
    const remote = s && s.navKind && s.navKind !== 0 ? { north: s.targetN - s.homeN, east: s.targetE - s.homeE } : null;
    const all = [...this.points, ...this.trail];
    if (drone) all.push(drone);
    if (this.target) all.push(this.target);
    if (remote) all.push(remote);
    this.range = mapRange(all);
    const at = (p) => metresToMap(p.north, p.east, size, this.range);
    const line = css.getPropertyValue("--line") || "#888", fg = css.getPropertyValue("--fg") || "#000", muted = css.getPropertyValue("--muted") || "#666";
    const accent = css.getPropertyValue("--accent") || "#25f", danger = css.getPropertyValue("--danger") || "#d22", ok = css.getPropertyValue("--ok") || "#2a2";
    g.clearRect(0, 0, size, size);
    g.lineWidth = 1;
    g.strokeStyle = line;
    g.fillStyle = muted;
    g.font = "11px system-ui, sans-serif";
    const step = this.range > 100 ? 50 : this.range > 40 ? 20 : 10;
    for (let r = step; r <= this.range; r += step) {
      const rp = (r * size) / (2 * this.range);
      g.beginPath();
      g.arc(size / 2, size / 2, rp, 0, 2 * Math.PI);
      g.stroke();
      g.fillText(`${r} m`, size / 2 + 3, size / 2 - rp + 11);
    }
    g.fillText("north is up", 6, 14);
    // where it has been
    g.strokeStyle = muted;
    g.beginPath();
    this.trail.forEach((p, i) => { const q = at(p); if (i) g.lineTo(q.x, q.y); else g.moveTo(q.x, q.y); });
    g.stroke();
    // the mission
    g.strokeStyle = accent;
    g.fillStyle = accent;
    g.beginPath();
    this.points.forEach((p, i) => { const q = at(p); if (i) g.lineTo(q.x, q.y); else g.moveTo(q.x, q.y); });
    g.stroke();
    this.points.forEach((p, i) => {
      const q = at(p);
      g.beginPath();
      g.arc(q.x, q.y, 4, 0, 2 * Math.PI);
      g.fill();
      g.fillText(String(i + 1), q.x + 6, q.y - 4);
    });
    // the point clicked, and where the drone is going now
    for (const [p, colour] of [[this.target, accent], [remote, danger]]) {
      if (!p) continue;
      const q = at(p);
      g.strokeStyle = colour;
      g.lineWidth = 2;
      g.beginPath();
      g.moveTo(q.x - 6, q.y - 6); g.lineTo(q.x + 6, q.y + 6);
      g.moveTo(q.x + 6, q.y - 6); g.lineTo(q.x - 6, q.y + 6);
      g.stroke();
      g.lineWidth = 1;
    }
    // home
    g.fillStyle = ok;
    g.fillRect(size / 2 - 5, size / 2 - 5, 10, 10);
    // the drone, pointing the way it faces
    if (drone && s) {
      const q = at(drone), a = (s.yaw * Math.PI) / 180;
      g.fillStyle = fg;
      g.beginPath();
      g.moveTo(q.x + 9 * Math.sin(a), q.y - 9 * Math.cos(a));
      g.lineTo(q.x + 5 * Math.sin(a + 2.4), q.y - 5 * Math.cos(a + 2.4));
      g.lineTo(q.x + 5 * Math.sin(a - 2.4), q.y - 5 * Math.cos(a - 2.4));
      g.closePath();
      g.fill();
    }
  }
}

// one line for the panel: what the drone is doing by itself
export function navText(s) {
  if (!s || s.navKind === undefined) return "-";
  if (!s.navKind) return s.armed ? "pilot is flying" : "on the ground";
  const left = s.navStage === 5 ? "" : `, ${s.navDist.toFixed(1)} m to go`;
  const point = s.navKindName === "MISSION" ? ` point ${s.wpIndex + 1}` : "";
  return `${s.navKindName}${point}: ${s.navStageName}${left}`;
}
