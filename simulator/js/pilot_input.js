// pilot_input.js - keyboard and gamepad -> stick values + commands for the flight controller.
//
// Sign convention (HIL_PROTOCOL.md): roll + = right, pitch - = forward (nose down), yaw + = clockwise (turn right)
//
// Keyboard (hold keys, the sticks spring back when released):
//   W / S  forward / back       A / D  left / right        Q / E  turn left / right
//   R / F  climb / descend (also Arrow Up / Down)          Arrow Left / Right  turn
//   Enter  arm / disarm         T take-off     L land      M  manual-throttle mode on/off      Space  KILL
//   1 / 2 / 3  speed mode ECO / NORMAL / SPORT (G = next)    P  position hold on/off    K  stick style SPEED/ANGLE
//   hold Shift = full stick (otherwise the keys give 60 %)
// Gamepad (standard / Xbox layout, mode 2):
//   left stick: up/down = climb/descend, left/right = turn       right stick: forward/back/left/right
//   A (X on PlayStation) = take-off    B (O) = land    Start (Options) = arm/disarm
//   Y (triangle) = manual mode    X (square) = next speed mode    RB (R1) = position hold on/off
//   hold Back (Share) 0.5 s = KILL
// Keyboard and gamepad work together: a pressed movement key overrides the gamepad on that axis.
export const CODES = { ARM: 10, DISARM: 11, TAKEOFF: 12, LAND: 13, KILL: 16, ECO: 17, NORMAL: 18, SPORT: 19, POSHOLD: 20, STICK: 21 };
const SPEED_CODES = [CODES.ECO, CODES.NORMAL, CODES.SPORT];
// Thumbsticks often rest off-centre after use. Microsoft's guidance for Xbox sticks is a deadzone of 0.24-0.27:
// anything smaller reads a resting stick as a small push, and the drone drifts instead of holding its position.
export const PAD_DEADZONE = 0.25;
// The left stick has two jobs. A thumb pulling it straight down also pushes it a little sideways, which would turn the
// drone on its way down. While the stick moves mainly along one job, a side part smaller than this share is ignored.
export const PAD_CROSS_AXIS = 0.5;
const AXES = ["roll", "pitch", "yaw", "climb"];
const REST_SECONDS = 3;      // an off-centre value that stays this steady is a resting stick, not a pilot

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
    this.speedMode = 1;      // updated from telemetry (for "next speed mode")
    this.enabled = true;
    this.keyDriven = { roll: false, pitch: false, yaw: false, climb: false };
    this.manualKb = false;   // manual throttle was last set with the keys (they hold it; the pad springs back)
    this.padRaw = null;      // gamepad axes before the deadzone
    this.restVal = [0, 0, 0, 0];
    this.restT = [0, 0, 0, 0];
    this.padRest = 0;        // largest off-centre value a stick is resting at
  }

  getPads() { return navigator.getGamepads ? Array.from(navigator.getGamepads()).filter(Boolean) : []; }

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
      if (k === "1" || k === "2" || k === "3") this.commands.push(SPEED_CODES[+k - 1]);
      if (k === "g") this.nextSpeed();
      if (k === "p") this.commands.push(CODES.POSHOLD);
      if (k === "k") this.commands.push(CODES.STICK);
    });
    target.addEventListener("keyup", (e) => this.keys.delete(e.key.toLowerCase()));
    target.addEventListener("blur", () => this.keys.clear());
  }

  nextSpeed() { this.commands.push(SPEED_CODES[(this.speedMode + 1) % 3]); }

  toggleManual() {
    this.manual = !this.manual;
    this.manualCmd = 0.66;
    this.manualKb = false;
  }

  // hidden = the page is not visible: nobody is flying, so the sticks go to neutral (position hold takes over)
  update(dt, hidden = false) {
    if (hidden) this.keys.clear();
    const gp = hidden ? null : this.getPads().find((p) => p.connected && p.axes.length >= 4);
    let padT = null;
    this.padRaw = null;
    if (gp) {
      this.gamepadName = gp.id + (gp.mapping === "standard" ? "" : " (non-standard mapping)");
      const ax = (i) => shape(gp.axes[i] || 0, PAD_DEADZONE, 0.3);
      const lx = Math.abs(gp.axes[0] || 0), ly = Math.abs(gp.axes[1] || 0);
      padT = {
        yaw: lx < PAD_CROSS_AXIS * ly ? 0 : ax(0),
        climb: ly < PAD_CROSS_AXIS * lx ? 0 : -ax(1),
        roll: ax(2), pitch: ax(3),
      };
      this.padRaw = [0, 1, 2, 3].map((i) => gp.axes[i] || 0);
      this.trackRest(dt);
      const pressed = (i) => gp.buttons[i] && gp.buttons[i].pressed;
      const edge = (i) => pressed(i) && !this.prevButtons[i];
      if (edge(0)) this.commands.push(CODES.TAKEOFF);
      if (edge(1)) this.commands.push(CODES.LAND);
      if (edge(9)) this.commands.push(this.armed ? CODES.DISARM : CODES.ARM);
      if (edge(3)) this.toggleManual();
      if (edge(2)) this.nextSpeed();
      if (edge(5)) this.commands.push(CODES.POSHOLD);
      if (pressed(8)) { this.killHold += dt; if (this.killHold > 0.5) { this.commands.push(CODES.KILL); this.killHold = -10; } }
      else this.killHold = 0;
      this.prevButtons = gp.buttons.map((b) => b.pressed);
    } else {
      this.gamepadName = "";
      this.padRest = 0;
    }
    this.source = gp ? "gamepad" : "keyboard";
    const k = (n) => (this.keys.has(n) ? 1 : 0);
    const s = this.keys.has("shift") ? 1 : this.kbStrength;
    const keyT = {
      roll: (k("d") - k("a")) * s,
      pitch: (k("s") - k("w")) * s,
      yaw: (k("e") - k("q") + k("arrowright") - k("arrowleft")) * s,
      climb: Math.max(-1, Math.min(1, k("r") + k("arrowup") - k("f") - k("arrowdown"))),
    };
    if (this.manual) {
      if (keyT.climb !== 0) {
        this.manualKb = true;
        this.manualCmd = Math.min(1, Math.max(0.3, this.manualCmd + keyT.climb * 0.15 * dt));
      } else if (padT && (!this.manualKb || padT.climb !== 0)) {
        this.manualKb = false;
        this.manualCmd = 0.66 + 0.25 * padT.climb;
      }
    }
    // a pressed key overrides the pad on that axis, smoothed (0.2 s to full deflection) so the drone does not jerk
    const rate = 5 * dt;
    for (const a of AXES) {
      const pad = padT ? padT[a] : 0;
      if (keyT[a] !== 0 || this.keyDriven[a]) {
        const target = keyT[a] !== 0 ? keyT[a] : pad;
        this.sticks[a] += Math.max(-rate, Math.min(rate, target - this.sticks[a]));
        this.keyDriven[a] = keyT[a] !== 0 || this.sticks[a] !== target;
      } else {
        this.sticks[a] = pad;
      }
    }
    return this.output();
  }

  // finds a stick that stays at the same off-centre value: it is resting there, nobody is pushing it
  trackRest(dt) {
    let rest = 0;
    this.padRaw.forEach((v, i) => {
      if (Math.abs(v - this.restVal[i]) > 0.02) { this.restVal[i] = v; this.restT[i] = 0; }
      else this.restT[i] += dt;
      const a = Math.abs(this.restVal[i]);
      if (this.restT[i] >= REST_SECONDS && a < 0.6) rest = Math.max(rest, a);   // above 0.6 = held on purpose
    });
    this.padRest = rest;
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
