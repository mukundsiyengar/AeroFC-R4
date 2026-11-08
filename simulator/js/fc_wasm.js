// fc_wasm.js - runs the real firmware core (compiled to fc.wasm) inside the browser.
let modulePromise = null;

export function loadFcModule(url = new URL("../fc.wasm", import.meta.url)) {
  // "no-cache": always ask the server whether fc.wasm changed (a rebuilt firmware must never run an old copy)
  if (!modulePromise) modulePromise = fetch(url, { cache: "no-cache" }).then((r) => {
    if (!r.ok) throw new Error("cannot load fc.wasm (" + r.status + ")");
    return r.arrayBuffer();
  }).then((b) => WebAssembly.compile(b));
  return modulePromise;
}

// Minimal WASI: the core never prints or reads files; anything else answers "not supported" (52 = ENOSYS)
function wasiImports(getMem) {
  const handler = {
    get(_t, name) {
      if (name === "proc_exit") return (code) => { throw new Error("wasm exit " + code); };
      if (name === "fd_write") return (fd, iovs, iovsLen, nwritten) => {
        const dv = new DataView(getMem().buffer);
        let n = 0;
        for (let i = 0; i < iovsLen; i++) n += dv.getUint32(iovs + 8 * i + 4, true);
        dv.setUint32(nwritten, n, true);
        return 0;
      };
      if (name === "random_get") return () => 0;
      if (name === "clock_time_get") return (_id, _p, out) => { new DataView(getMem().buffer).setBigUint64(out, 0n, true); return 0; };
      return () => 52;
    },
  };
  return { wasi_snapshot_preview1: new Proxy({}, handler) };
}

export const BTN = { UP: 0, DOWN: 1, SELECT: 2, BACK: 3 };
export const EV = { SHORT: 1, LONG: 2, REPEAT: 3 };

export class WasmFc {
  static async create(storageKey = null) {
    const mod = await loadFcModule();
    let inst = null;
    inst = await WebAssembly.instantiate(mod, wasiImports(() => inst.exports.memory));
    const fc = new WasmFc(inst, storageKey);
    return fc;
  }

  constructor(inst, storageKey) {
    this.x = inst.exports;
    if (this.x._initialize) this.x._initialize();
    this.x.fc_init();
    this.storageKey = storageKey;
    this.inPtr = this.x.fc_in_ptr();
    this.outPtr = this.x.fc_out_ptr();
    this.fbPtr = this.x.fc_fb_ptr();
    this.setPtr = this.x.fc_settings_ptr();
    this.infoPtr = this.x.fc_info_ptr();
    this.loadedSettings = false;
    if (storageKey) this.loadSettings();
  }
  mem() { return new Uint8Array(this.x.memory.buffer); }

  feed(bytes, nowUs) {
    for (let off = 0; off < bytes.length; off += 2048) {
      const chunk = bytes.subarray(off, Math.min(off + 2048, bytes.length));
      this.mem().set(chunk, this.inPtr);
      this.x.fc_feed(chunk.length, nowUs >>> 0);
    }
  }
  tick(nowUs) { this.x.fc_tick(nowUs >>> 0); }
  poll(nowUs) {
    const n = this.x.fc_poll(nowUs >>> 0);
    return n > 0 ? this.mem().slice(this.outPtr, this.outPtr + n) : null;
  }
  button(btn, evt, nowUs) { this.x.fc_button(btn, evt, nowUs >>> 0); }
  render(nowUs) { return this.x.fc_render(nowUs >>> 0) ? this.mem().slice(this.fbPtr, this.fbPtr + 1024) : null; }
  displayOn() { return this.x.fc_display_on() === 1; }
  cmd(code, nowUs) { return this.x.fc_cmd(code, nowUs >>> 0); }
  info() {
    this.x.fc_info();
    return new Float32Array(this.x.memory.buffer.slice(this.infoPtr, this.infoPtr + 100 * 4));
  }
  paramCount() { return this.x.fc_param_count(); }
  paramName(i) {
    const p = this.x.fc_param_name(i), m = this.mem();
    let e = p;
    while (m[e]) e++;
    return new TextDecoder().decode(m.slice(p, e));
  }
  param(i) { return this.x.fc_param_get(i); }
  setParam(i, v) { this.x.fc_param_set(i, v); }
  cstr(ptr) {
    const m = this.mem();
    let e = ptr;
    while (m[e]) e++;
    return new TextDecoder().decode(m.slice(ptr, e));
  }
  helpTexts() {
    const out = [];
    for (let i = 0; i < this.x.fc_help_count(); i++) out.push({ title: this.cstr(this.x.fc_help_title(i)), text: this.cstr(this.x.fc_help_text(i)) });
    return out;
  }
  paramIndex(name) { for (let i = 0; i < this.paramCount(); i++) if (this.paramName(i) === name) return i; return -1; }
  set(name, v) { const i = this.paramIndex(name); if (i < 0) throw new Error("no param " + name); this.setParam(i, v); }
  get(name) { return this.param(this.paramIndex(name)); }

  // ---- settings persistence (the "flash" of the virtual board is localStorage)
  loadSettings() {
    try {
      const s = localStorage.getItem(this.storageKey);
      if (!s) return false;
      const bytes = Uint8Array.from(atob(s), (c) => c.charCodeAt(0));
      this.mem().set(bytes.subarray(0, 1024), this.setPtr);
      this.loadedSettings = this.x.fc_settings_unpack(bytes.length) === 1;
      return this.loadedSettings;
    } catch { return false; }
  }
  // call regularly: performs saves / loads requested from the OLED menu
  serviceStorage() {
    if (!this.storageKey) return;
    if (this.x.fc_save_requested()) {
      let ok = -1;
      try {
        const n = this.x.fc_settings_pack();
        const bytes = this.mem().slice(this.setPtr, this.setPtr + n);
        localStorage.setItem(this.storageKey, btoa(String.fromCharCode(...bytes)));
        ok = 1;
      } catch { ok = -1; }
      this.x.fc_save_done(ok);
    }
    if (this.x.fc_load_requested()) this.x.fc_load_done(this.loadSettings() ? 1 : -1);
  }
}
