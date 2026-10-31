// page_status.js - turns what the page knows into one plain sentence: do the commands reach the flight controller, and
// what are the throttle stick and the motors doing. No drawing here, so the tests can check every answer.

// i: mode, bridgeSeen (the Gazebo bridge answered), relay, connectedFor (s), postStatus (HTTP answer to the last command,
// 0 = none), telemetryAge (s), pilotHz (commands per second the flight controller counts), rcCut, hidden, focused, pad
export function linkStatus(i) {
  if (i.mode !== "gazebo") {
    if (!i.bridgeSeen) return null;                    // no Gazebo running: nothing to warn about
    const doing = i.mode === "software" ? "This page is flying its own built-in copy, so your keys do not reach Gazebo."
      : i.mode === "arduino" ? "This page is flying the board with its own physics, so your keys do not reach Gazebo."
      : "This page is not connected to it yet, so your keys reach nothing.";
    return { level: "bad", text: "Gazebo is running. " + doing, action: "gazebo" };
  }
  if (i.rcCut) return { level: "wait", text: "Commands stopped: you cut the RC link (card 4)." };
  if (i.hidden) return { level: "wait", text: "This page is hidden, so it sends centred sticks. Bring it back to fly." };
  if (!i.postStatus) {
    return i.connectedFor < 60
      ? { level: "wait", text: "Waiting for Gazebo and the bridge to start..." }
      : { level: "bad", text: `No answer from the bridge at ${i.relay}. Is the Gazebo window (launcher 3 or 4) still open?` };
  }
  if (i.postStatus !== 200) {
    return i.connectedFor < 60
      ? { level: "wait", text: "The bridge is running. Waiting for the flight controller to answer..." }
      : { level: "bad", text: "The bridge is running, but the flight controller does not answer. Is the Arduino given to Ubuntu (launcher 2)?" };
  }
  if (i.telemetryAge > 2) return { level: "bad", text: "The bridge takes the commands, but no telemetry comes back from the flight controller." };
  if (i.pilotHz > 40) return { level: "bad", text: `Two pilots: the flight controller gets ${i.pilotHz} commands per second. Close the other simulator page.` };
  if (i.pilotHz < 10) return { level: "wait", text: `Commands are on their way: the flight controller counts ${i.pilotHz} per second so far...` };
  const flowing = `Commands reach the flight controller: ${i.pilotHz} per second.`;
  if (!i.focused && !i.pad) return { level: "wait", text: flowing + " Click on this page once: keys go to the window you clicked last." };
  return { level: "ok", text: flowing };
}

// the throttle stick the flight controller really received, and the motor power compared with hover
export function throttleView(s) {
  if (!s) return { stick: 0, stickText: "-", power: null, powerText: "-" };
  const t = s.rcThr;
  let stick = 0, stickText = "no stick received";
  if (t >= 1.5 && t <= 3.5) {                          // climb stick in altitude hold
    stick = (t - 2.5) * 2;
    const pct = Math.round(Math.abs(stick) * 100);
    stickText = pct === 0 ? "centre: hold height" : `${stick > 0 ? "up" : "down"} ${pct} %`;
  } else if (t > 0 && t <= 1) {                        // manual throttle: 0.66 is about hover
    stick = Math.max(-1, Math.min(1, (t - 0.66) * 3));
    stickText = `manual ${Math.round(t * 100)} %`;
  }
  const avg = s.motors ? s.motors.reduce((a, m) => a + m, 0) / s.motors.length : 0;
  const power = s.armed && s.airborne ? Math.round((avg - s.hoverCmd) * 100) : null;
  return { stick, stickText, power, powerText: power === null ? "-" : `${power > 0 ? "+" : ""}${power} %` };
}
