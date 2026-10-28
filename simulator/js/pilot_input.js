// pilot_input.js - keyboard and gamepad -> stick values + commands for the flight controller.
//
// Sign convention (HIL_PROTOCOL.md): roll + = right, pitch - = forward (nose down), yaw + = clockwise (turn right)
//
// Keyboard (hold keys, the sticks spring back when released):
//   W / S  forward / back       A / D  left / right        Q / E  turn left / right
//   R / F  climb / descend (also Arrow Up / Down)          Arrow Left / Right  turn
//   Enter  arm / disarm         T take-off     L land      M  manual-throttle mode on/off      Space  KILL
// Gamepad (standard / Xbox layout, mode 2):
//   left stick: up/down = climb/descend, left/right = turn       right stick: forward/back/left/right
//   A (X on PlayStation) = take-off    B (O) = land    Start (Options) = arm/disarm
//   Y (triangle) = manual mode    hold Back (Share) 0.5 s = KILL
export const CODES = { ARM: 10, DISARM: 11, TAKEOFF: 12, LAND: 13, KILL: 16 };

const shape = (v, db, expo) => {
  const a = Math.abs(v);
  if (a <= db) return 0;
  const x = Math.sign(v) * (a - db) / (1 - db);
  return x * (1 - expo) + x * x * x * expo;
};

export class Pilot {
  constructor() {
    this.keys = new Set();
    this.sticks = { roll: 0, pitch: 0, yaw: 0, climb: 0 };
    this.manual = false;
    this.manualCmd = 0.66;
    this.kbStrength = 0.6;   // keyboard stick deflection (fraction of full stick)
    this.commands = [];
    this.source = "keyboard";
    this.gamepadName = "";
    this.prevButtons = [];
    this.killHold = 0;
    this.armed = false;      // updated from telemetry (for the arm / disarm toggle)
    this.enabled = true;
  }

  attach(target = window) {
    target.addEventListener("keydown", (e) => {
      if (!this.enabled || e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
      const k = e.key.toLowerCase();
      if (["w", "a", "s", "d", "q", "e", "r", "f", "arrowup", "arrowdown", "arrowleft", "arrowright", " ", "enter"].includes(k)) e.preventDefault();
      if (e.repeat) return;
      this.keys.add(k);
      if (k === "enter") this.commands.push(this.armed ? CODES.DISARM : CODES.ARM);
      if (k === "t") this.commands.push(CODES.TAKEOFF);
      if (k === "l") this.commands.push(CODES.LAND);
      if (k === " ") this.commands.push(CODES.KILL);
      if (k === "m") this.toggleManual();
    });
    target.addEventListener("keyup", (e) => this.keys.delete(e.key.toLowerCase()));
    target.addEventListener("blur", () => this.keys.clear());
  }

  toggleManual() {
    this.manual = !this.manual;
    this.manualCmd = 0.66;
  }

  update(dt) {
    const pads = navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : [];
    const gp = pads.find((p) => p.connected && p.axes.length >= 4);
    let target;
    if (gp) {
      this.source = "gamepad";
      this.gamepadName = gp.id + (gp.mapping === "standard" ? "" : " (non-standard mapping)");
      const ax = (i) => shape(gp.axes[i] || 0, 0.08, 0.3);
      target = { yaw: ax(0), climb: -ax(1), roll: ax(2), pitch: ax(3) };
      const pressed = (i) => gp.buttons[i] && gp.buttons[i].pressed;
      const edge = (i) => pressed(i) && !this.prevButtons[i];
      if (edge(0)) this.commands.push(CODES.TAKEOFF);
      if (edge(1)) this.commands.push(CODES.LAND);
      if (edge(9)) this.commands.push(this.armed ? CODES.DISARM : CODES.ARM);
      if (edge(3)) this.toggleManual();
      if (pressed(8)) { this.killHold += dt; if (this.killHold > 0.5) { this.commands.push(CODES.KILL); this.killHold = -10; } }
      else this.killHold = 0;
      this.prevButtons = gp.buttons.map((b) => b.pressed);
      if (this.manual) this.manualCmd = 0.66 + 0.25 * target.climb;
      Object.assign(this.sticks, target);
    } else {
      this.source = "keyboard";
      this.gamepadName = "";
      const k = (n) => (this.keys.has(n) ? 1 : 0);
      const s = this.kbStrength;
      target = {
        roll: (k("d") - k("a")) * s,
        pitch: (k("s") - k("w")) * s,
        yaw: (k("e") - k("q") + k("arrowright") - k("arrowleft")) * s,
        climb: Math.max(-1, Math.min(1, k("r") + k("arrowup") - k("f") - k("arrowdown"))),
      };
      if (this.manual) this.manualCmd = Math.min(1, Math.max(0.3, this.manualCmd + target.climb * 0.15 * dt));
      // smooth the keyboard "sticks" (0.2 s to full deflection) so the drone does not jerk
      const rate = 5 * dt;
      for (const a of ["roll", "pitch", "yaw", "climb"]) {
        const d = target[a] - this.sticks[a];
        this.sticks[a] += Math.max(-rate, Math.min(rate, d));
      }
    }
    return this.output();
  }

  // values for the CONTROL_INPUT packet (our extension: throttle 2..3 = climb stick in altitude hold)
  output() {
    const s = this.sticks;
    return {
      thr: this.manual ? this.manualCmd : 2.5 + 0.5 * s.climb,
      roll: s.roll, pitch: s.pitch, yaw: s.yaw,
    };
  }

  takeCommands() { const c = this.commands; this.commands = []; return c; }
}
