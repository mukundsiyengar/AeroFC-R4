// vision.js - the laptop's cameras, optional and off by default: gesture control on the built-in camera, object detection on
// the C270. They run here in the page with MediaPipe. Turning one off stops its camera completely. MediaPipe is only loaded when
// one is turned on, so with both off nothing here costs anything.
import { GestureGate, GESTURE_ACTIONS, chooseCamera } from "./vision_logic.js";

const VENDOR = new URL("../vendor/", import.meta.url).href;
let tasks = null;

async function loadMediaPipe() {
  if (!tasks) {
    const mp = await import(VENDOR + "mediapipe/vision_bundle.mjs");
    tasks = { mp, files: await mp.FilesetResolver.forVisionTasks(VENDOR + "mediapipe/wasm") };
  }
  return tasks;
}

export async function makeGestureRecognizer() {
  const { mp, files } = await loadMediaPipe();
  return mp.GestureRecognizer.createFromOptions(files, {
    baseOptions: { modelAssetPath: VENDOR + "models/gesture_recognizer.task", delegate: "CPU" },
    runningMode: "VIDEO", numHands: 1,
  });
}

export async function makeObjectDetector() {
  const { mp, files } = await loadMediaPipe();
  return mp.ObjectDetector.createFromOptions(files, {
    baseOptions: { modelAssetPath: VENDOR + "models/efficientdet_lite0.tflite", delegate: "CPU" },
    runningMode: "VIDEO", scoreThreshold: 0.45, maxResults: 8,
  });
}

// the same models on a still picture: for the tests, which have no camera
export async function imageGesture(img) {
  const { mp, files } = await loadMediaPipe();
  const r = await mp.GestureRecognizer.createFromOptions(files, { baseOptions: { modelAssetPath: VENDOR + "models/gesture_recognizer.task", delegate: "CPU" }, runningMode: "IMAGE", numHands: 1 });
  const out = r.recognize(img);
  r.close();
  const g = out.gestures[0] && out.gestures[0][0];
  return g ? { name: g.categoryName, score: g.score } : { name: "", score: 0 };
}
export async function imageObjects(img) {
  const { mp, files } = await loadMediaPipe();
  const d = await mp.ObjectDetector.createFromOptions(files, { baseOptions: { modelAssetPath: VENDOR + "models/efficientdet_lite0.tflite", delegate: "CPU" }, runningMode: "IMAGE", scoreThreshold: 0.45, maxResults: 8 });
  const out = d.detect(img);
  d.close();
  return out.detections.map((x) => ({ name: x.categories[0].categoryName, score: x.categories[0].score, box: x.boundingBox }));
}

// One camera, one job. work(video, nowMs, draw) runs at most `fps` times a second.
class CameraJob {
  constructor(video, overlay, want, fps) {
    this.video = video;
    this.overlay = overlay;
    this.want = want;
    this.period = 1000 / fps;
    this.stream = null;
    this.running = false;
    this.next = 0;
    this.msUsed = 0;
  }

  async start(work, deviceId) {
    let id = deviceId;
    if (!id) {
      // the first request gets the permission, and only after that does the browser name the cameras
      const first = await navigator.mediaDevices.getUserMedia({ video: true });
      first.getTracks().forEach((t) => t.stop());
      const cam = chooseCamera(await navigator.mediaDevices.enumerateDevices(), this.want);
      if (!cam) throw new Error("no camera found");
      id = cam.deviceId;
    }
    this.stream = await navigator.mediaDevices.getUserMedia({ video: { deviceId: { exact: id }, width: 640, height: 480 } });
    this.video.srcObject = this.stream;
    await this.video.play();
    this.label = this.stream.getVideoTracks()[0].label;
    this.running = true;
    const loop = () => {
      if (!this.running) return;
      const now = performance.now();
      if (now >= this.next && this.video.readyState >= 2) {
        this.next = now + this.period;
        const t0 = performance.now();
        work(this.video, now, this.overlay);
        this.msUsed = performance.now() - t0;
      }
      this.video.requestVideoFrameCallback ? this.video.requestVideoFrameCallback(loop) : requestAnimationFrame(loop);
    };
    loop();
  }

  stop() {
    this.running = false;
    if (this.stream) this.stream.getTracks().forEach((t) => t.stop());
    this.stream = null;
    this.video.srcObject = null;
    const g = this.overlay.getContext("2d");
    g.clearRect(0, 0, this.overlay.width, this.overlay.height);
  }
}

function fitOverlay(video, overlay) {
  if (overlay.width !== video.videoWidth) { overlay.width = video.videoWidth; overlay.height = video.videoHeight; }
  const g = overlay.getContext("2d");
  g.clearRect(0, 0, overlay.width, overlay.height);
  return g;
}

export class GesturePanel {
  // sendCode(code) sends a command to the drone; report(text) writes what it sees on the page; log(text) writes in the log
  constructor(video, overlay, sendCode, report, log) {
    this.job = new CameraJob(video, overlay, "own", 12);
    this.gate = new GestureGate();
    Object.assign(this, { sendCode, report, log, recognizer: null });
  }

  async start() {
    this.report("loading...");
    this.recognizer = this.recognizer || await makeGestureRecognizer();
    this.gate.reset();
    await this.job.start((v, now, ov) => this.step(v, now, ov));
    this.log(`Gestures on, camera: ${this.job.label}`);
  }

  step(video, now, overlay) {
    const out = this.recognizer.recognizeForVideo(video, now);
    const g = out.gestures[0] && out.gestures[0][0];
    const name = g ? g.categoryName : "", score = g ? g.score : 0;
    const code = this.gate.update(name, score, now / 1000);
    const ctx = fitOverlay(video, overlay);
    ctx.fillStyle = "#22c55e";
    for (const p of out.landmarks[0] || []) ctx.fillRect(p.x * overlay.width - 3, p.y * overlay.height - 3, 6, 6);
    const action = GESTURE_ACTIONS[name];
    if (code !== null) {
      this.sendCode(code);
      this.log(`Gesture ${name}: ${action.text}`);
    }
    const pct = Math.round(this.gate.progress(now / 1000) * 100);
    this.report(action ? `${name}: ${action.text}${pct ? ` (hold ${pct} %)` : " (done, let go)"}` : name && name !== "None" ? `${name}: does nothing` : "no gesture");
  }

  stop() {
    this.job.stop();
    this.gate.reset();
    this.report("off");
    this.log("Gestures off, camera stopped");
  }
}

export class ObjectPanel {
  constructor(video, overlay, report, log) {
    this.job = new CameraJob(video, overlay, "c270", 5);
    Object.assign(this, { report, log, detector: null });
  }

  async start() {
    this.report("loading...");
    this.detector = this.detector || await makeObjectDetector();
    await this.job.start((v, now, ov) => this.step(v, now, ov));
    this.log(`Object detection on, camera: ${this.job.label}`);
  }

  step(video, now, overlay) {
    const found = this.detector.detectForVideo(video, now).detections;
    const ctx = fitOverlay(video, overlay);
    ctx.lineWidth = 3;
    ctx.font = "18px system-ui, sans-serif";
    for (const d of found) {
      const b = d.boundingBox, c = d.categories[0];
      ctx.strokeStyle = "#f59e0b";
      ctx.strokeRect(b.originX, b.originY, b.width, b.height);
      ctx.fillStyle = "#f59e0b";
      ctx.fillText(`${c.categoryName} ${Math.round(c.score * 100)} %`, b.originX + 4, Math.max(18, b.originY - 4));
    }
    this.report(found.length ? found.map((d) => d.categories[0].categoryName).join(", ") : "nothing recognised");
  }

  stop() {
    this.job.stop();
    this.report("off");
    this.log("Object detection off, camera stopped");
  }
}
