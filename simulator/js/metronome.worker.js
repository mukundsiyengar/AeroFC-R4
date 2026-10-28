// metronome.worker.js - steady 4 ms ticks.  Timers in a background tab are slowed down by the browser,
// timers in a worker much less, so the simulation keeps its 50 Hz sensor rate even if you switch windows.
let id = null;
onmessage = (e) => {
  if (e.data === "start" && id === null) id = setInterval(() => postMessage(0), 4);
  if (e.data === "stop" && id !== null) { clearInterval(id); id = null; }
};
