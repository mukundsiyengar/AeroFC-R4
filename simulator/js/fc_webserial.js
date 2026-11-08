// fc_webserial.js - talks to the real Arduino over USB with the browser's Web Serial API (Edge / Chrome).
export class SerialLink {
  static supported() { return "serial" in navigator; }

  constructor() {
    this.port = null;
    this.writer = null;
    this.reader = null;
    this.onData = null;
    this.onClose = null;
    this.bytesIn = 0;
    this.bytesOut = 0;
  }

  async open(baudRate = 115200) {
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate, bufferSize: 65536 });
    // the firmware works without DTR, but set it like pyserial does
    try { await this.port.setSignals({ dataTerminalReady: true, requestToSend: true }); } catch (_) { /* not supported by every adapter */ }
    this.writer = this.port.writable.getWriter();
    this.readLoop();
    const info = this.port.getInfo ? this.port.getInfo() : {};
    return info;
  }

  write(bytes) {
    if (!this.writer) return;
    this.bytesOut += bytes.length;
    this.writer.write(bytes).catch(() => {});
  }

  async readLoop() {
    while (this.port && this.port.readable) {
      this.reader = this.port.readable.getReader();
      try {
        for (;;) {
          const { value, done } = await this.reader.read();
          if (done) break;
          if (value) { this.bytesIn += value.length; if (this.onData) this.onData(value); }
        }
      } catch (e) {
        break;   // unplugged
      } finally {
        try { this.reader.releaseLock(); } catch (_) { }
      }
    }
    if (this.onClose) this.onClose();
  }

  async close() {
    try { if (this.reader) await this.reader.cancel(); } catch (_) { }
    try { if (this.writer) { this.writer.releaseLock(); } } catch (_) { }
    try { if (this.port) await this.port.close(); } catch (_) { }
    this.port = this.writer = this.reader = null;
  }
}
