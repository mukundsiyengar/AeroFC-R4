// fc_wasm_api.cpp - exports the UNMODIFIED firmware core (firmware/AeroFC_R4_v3/src/core) as a WebAssembly module,
// so the browser simulator flies exactly the code that runs on the Arduino.
#include <string.h>
#include "../../firmware/AeroFC_R4_v3/src/core/fc_app.h"
#include "../../firmware/AeroFC_R4_v3/src/core/ui_menu.h"

static FcApp app;
static UiMenu ui;
static UiPlatform plat;
static unsigned char inBuf[2048];
static unsigned char outBuf[2048];
static unsigned char fb[1024];
static unsigned char settingsBuf[1024];
static float info[80];

#define EXPORT(n) extern "C" __attribute__((export_name(n)))

EXPORT("fc_init") void fc_init() {
  app.init();
  plat.resetCause = "simulator";
  ui.init(&app, &plat);
}
EXPORT("fc_in_ptr") unsigned char *fc_in_ptr() { return inBuf; }
EXPORT("fc_out_ptr") unsigned char *fc_out_ptr() { return outBuf; }
EXPORT("fc_fb_ptr") unsigned char *fc_fb_ptr() { return fb; }
EXPORT("fc_settings_ptr") unsigned char *fc_settings_ptr() { return settingsBuf; }
EXPORT("fc_info_ptr") float *fc_info_ptr() { return info; }

// feed n bytes from inBuf
EXPORT("fc_feed") void fc_feed(int n, unsigned int nowUs) { app.feed(inBuf, (size_t)n, nowUs); }
EXPORT("fc_tick") void fc_tick(unsigned int nowUs) { app.tick(nowUs); }
// copies all pending USB packets into outBuf, returns the number of bytes
EXPORT("fc_poll") int fc_poll(unsigned int nowUs) {
  size_t total = 0;
  for (int guard = 0; guard < 16; guard++) {
    const size_t n = app.nextUsbPacket(outBuf + total, sizeof(outBuf) - total, nowUs);
    if (!n) break;
    total += n;
  }
  return (int)total;
}

// ---- virtual OLED + buttons
EXPORT("fc_button") void fc_button(int btn, int evt, unsigned int nowUs) { ui.onButton((uint8_t)btn, (uint8_t)evt, nowUs); }
// renders into fb (SSD1306 page layout: 8 pages x 128 columns, LSB = top pixel); returns 1 if it changed
EXPORT("fc_render") int fc_render(unsigned int nowUs) { return ui.render(fb, nowUs) ? 1 : 0; }
// help texts (for the automated "everything fits on the OLED" test)
EXPORT("fc_help_count") int fc_help_count() { return UiMenu::helpCount(); }
EXPORT("fc_help_title") const char *fc_help_title(int i) { return UiMenu::helpTitle(i); }
EXPORT("fc_help_text") const char *fc_help_text(int i) { return UiMenu::helpTextAt(i); }

// ---- settings: the browser keeps them in localStorage
EXPORT("fc_settings_pack") int fc_settings_pack() { return (int)app.packSettings(settingsBuf, sizeof(settingsBuf)); }
EXPORT("fc_settings_unpack") int fc_settings_unpack(int n) {
  const bool ok = app.unpackSettings(settingsBuf, (size_t)n);
  if (ok) app.bootFromDefaults = false;
  return ok ? 1 : 0;
}
EXPORT("fc_load_requested") int fc_load_requested() { return app.loadRequested ? 1 : 0; }
EXPORT("fc_load_done") void fc_load_done(int ok) {
  app.loadRequested = false;
  app.lastLoadResult = (int8_t)ok;
}
EXPORT("fc_save_requested") int fc_save_requested() { return app.saveRequested ? 1 : 0; }
EXPORT("fc_save_done") void fc_save_done(int ok) {
  app.saveRequested = false;
  app.lastSaveResult = (int8_t)ok;
}

// ---- parameters (for the simulator's panels)
EXPORT("fc_param_count") int fc_param_count() { return paramsCount(); }
EXPORT("fc_param_get") float fc_param_get(int i) { return paramGet(app.params, i); }
EXPORT("fc_param_set") void fc_param_set(int i, float v) { paramSet(app.params, i, v); }
EXPORT("fc_param_name") const char *fc_param_name(int i) { return paramDef(i).name; }
EXPORT("fc_param_group") int fc_param_group(int i) { return paramDef(i).group; }

// ---- commands from the simulator panel (same codes as the pilot extension)
EXPORT("fc_cmd") int fc_cmd(int code, unsigned int nowUs) {
  switch (code) {
    case 10: return app.core.cmdArm(nowUs);
    case 11: app.core.cmdDisarm(R_USER); return 0;
    case 12: return app.core.cmdTakeoff(nowUs);
    case 13: app.core.cmdLand(); return 0;
    case 16: app.core.cmdKill(); return 0;
    default: return -1;
  }
}

// ---- internal state for plots / tests (estimates, setpoints, debug)
EXPORT("fc_info") void fc_info() {
  const FlightCore &c = app.core;
  float *o = info;
  o[0] = c.ekf.roll(); o[1] = c.ekf.pitch(); o[2] = c.ekf.yaw();
  o[3] = c.altf.alt; o[4] = c.altf.vz; o[5] = c.altTarget;
  o[6] = c.rollSp; o[7] = c.pitchSp; o[8] = c.att.yawRateSp; o[9] = c.att.headingTarget;
  o[10] = (float)c.state; o[11] = (float)c.mode; o[12] = (float)c.reason; o[13] = c.airborne ? 1.0f : 0.0f;
  o[14] = c.collective; o[15] = c.torque.x; o[16] = c.torque.y; o[17] = c.torque.z;
  for (int i = 0; i < 4; i++) o[18 + i] = c.motor[i];
  o[22] = c.ekf.bias.x; o[23] = c.ekf.bias.y; o[24] = c.ekf.bias.z;
  o[25] = c.ekf.magHealthy ? 1.0f : 0.0f; o[26] = (float)c.ekf.magFused; o[27] = c.ekf.initialized() ? 1.0f : 0.0f;
  o[28] = c.altc.integ; o[29] = c.altc.vzSp; o[30] = (float)app.session; o[31] = c.imuHz;
  o[32] = c.att.rateSp.x; o[33] = c.att.rateSp.y; o[34] = c.att.rateSp.z;
  o[35] = (float)c.lastArmResult; o[36] = c.crashed ? 1.0f : 0.0f; o[37] = c.pilot.streamSeen ? 1.0f : 0.0f;
  o[38] = c.pilot.linkLost ? 1.0f : 0.0f; o[39] = (float)app.parser.crcErrors; o[40] = c.tiltDeg;
  o[41] = (float)app.txActuators; o[42] = (float)app.txHeartbeats; o[43] = (float)app.rxCount[0];
  o[44] = c.homeNorth; o[45] = c.homeEast; o[46] = c.ekf.accRejects; o[47] = c.ekf.magRejects;
  o[48] = c.sensorsOk ? 1.0f : 0.0f; o[49] = c.geofence ? 1.0f : 0.0f;
  o[50] = c.gpsAidOk ? 1.0f : 0.0f; o[51] = c.aidYawOffset;
  // position hold
  o[52] = c.posReady ? 1.0f : 0.0f; o[53] = c.posActive ? 1.0f : 0.0f; o[54] = (float)c.posState;
  o[55] = c.posFailed ? 1.0f : 0.0f;
  o[56] = c.pos.kn.p; o[57] = c.pos.ke.p; o[58] = c.pos.kn.v; o[59] = c.pos.ke.v;
  o[60] = c.posCtl.holdN; o[61] = c.posCtl.holdE; o[62] = c.posCtl.dist;
  o[63] = c.posCtl.iN; o[64] = c.posCtl.iE; o[65] = c.posCtl.vSpN; o[66] = c.posCtl.vSpE;
  o[67] = c.pos.sigma(); o[68] = c.ceilingAbs; o[69] = c.atCeiling ? 1.0f : 0.0f;
  o[70] = app.params.speedMode; o[71] = c.groundAlt;
}
