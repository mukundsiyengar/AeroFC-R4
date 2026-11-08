// vision_logic.js - the rules for gesture control, with no camera in them, so the tests can check every case.
//
// A gesture is a command only if it is clear and held on purpose: a hand passing the camera must never fly the drone.
export const GESTURE_HOLD_S = 1.0;       // hold it this long
export const GESTURE_MIN_SCORE = 0.6;    // how sure the recogniser must be
const GESTURE_GONE_S = 0.5;              // this long without it, and it counts as let go

// What each gesture does.  Nothing here can stop the motors: a wrong guess must never drop the drone.
export const GESTURE_ACTIONS = {
  Thumb_Up: { code: 12, text: "take off" },
  Thumb_Down: { code: 13, text: "land" },
  Open_Palm: { code: 28, text: "stop the trip, hold here" },
  Victory: { code: 22, text: "return home" },
};

export class GestureGate {
  constructor() { this.reset(); }

  reset() {
    this.name = "";         // the gesture being held
    this.since = 0;         // when it started (s)
    this.lastSeen = -99;    // when it was last seen (s)
    this.fired = false;     // it has already done its command: let go first
  }

  // name, score from the recogniser ("" = nothing seen), t in seconds. Returns a command code once, or null.
  update(name, score, t) {
    const good = !!GESTURE_ACTIONS[name] && score >= GESTURE_MIN_SCORE;
    if (good) {
      if (name !== this.name) {
        this.name = name;
        this.since = t;
        this.fired = false;
      }
      this.lastSeen = t;
    } else if (this.name && t - this.lastSeen > GESTURE_GONE_S) {
      this.reset();
    }
    if (good && !this.fired && t - this.since >= GESTURE_HOLD_S) {
      this.fired = true;
      return GESTURE_ACTIONS[name].code;
    }
    return null;
  }

  // 0..1: how far through the hold it is, for the page
  progress(t) {
    if (!this.name || this.fired) return 0;
    return Math.min(1, (t - this.since) / GESTURE_HOLD_S);
  }
}

// The cameras the page can offer: the C270 for object detection, the laptop's own for gestures.
// Before the user allows the camera, browsers give no labels, so the first choice is all there is.
export function chooseCamera(devices, want) {
  const cams = devices.filter((d) => d.kind === "videoinput");
  if (!cams.length) return null;
  const c270 = cams.find((d) => /c270/i.test(d.label));
  const other = cams.find((d) => !/c270/i.test(d.label));
  return want === "c270" ? (c270 || other) : (other || c270);
}
