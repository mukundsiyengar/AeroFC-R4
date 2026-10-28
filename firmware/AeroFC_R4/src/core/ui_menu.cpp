#include "ui_menu.h"
#include <string.h>

static const int ROWS = 7;                 // list rows under the title bar (8 px each)
static const int HOME_PAGES = 7;
static const int MENU_FIXED_TOP = 3;       // Flight, Calibrate, Storage, then the parameter groups, then About
static const int MENU_COUNT = MENU_FIXED_TOP + PG_COUNT + 1;

static const char *menuName(int i) {
  if (i == 0) return "Flight";
  if (i == 1) return "Calibrate";
  if (i == 2) return "Storage";
  if (i < MENU_FIXED_TOP + PG_COUNT) return paramGroupName((uint8_t)(i - MENU_FIXED_TOP));
  return "About";
}

static const char *sessionName(const FcApp &a, uint32_t now) {
  if (a.linkActive(now)) return a.session == SES_RUNNING ? "PC: RUNNING" : "PC: DATA";
  switch (a.session) {
    case SES_HANDSHAKE: return "PC: HANDSHAKE";
    case SES_RUNNING: return "PC: LOST!";
    case SES_STOPPED: return "PC: STOPPED";
    default: return "PC: WAIT PC";
  }
}

void UiMenu::init(FcApp *a) {
  app = a;
  screen = SC_HOME;
  homePage = 0;
  memset(altHist, 0, sizeof(altHist));
  toastMsg[0] = 0;
  dirty = true;
}

void UiMenu::toast(const char *msg, uint32_t nowUs, uint32_t ms) {
  strncpy(toastMsg, msg, sizeof(toastMsg) - 1);
  toastMsg[sizeof(toastMsg) - 1] = 0;
  toastUntil = nowUs + ms * 1000u;
  if (toastUntil == 0) toastUntil = 1;
  dirty = true;
}

// ------------------------------------------------------------------ buttons
void UiMenu::onButton(uint8_t btn, uint8_t evt, uint32_t nowUs) {
  now = nowUs;
  dirty = true;
  if (evt == EV_REPEAT) repeatCount++;
  else repeatCount = 0;
  switch (screen) {
    case SC_HOME: homeButton(btn, evt); break;
    case SC_MENU: menuButton(btn, evt); break;
    case SC_FLIGHT: flightButton(btn, evt); break;
    case SC_PARAMS: paramsButton(btn, evt); break;
    case SC_CAL: calButton(btn, evt); break;
    case SC_STORAGE: storageButton(btn, evt); break;
    default: if (btn == BTN_BACK || btn == BTN_SELECT) screen = SC_MENU; break;
  }
}

void UiMenu::armToggle() {
  FlightCore &c = app->core;
  if (!c.armed()) {
    const int8_t r = c.cmdArm(now);
    char m[22] = "ARM: ";
    strncat(m, armResultName(r), sizeof(m) - 6);
    toast(r == ARM_OK ? "ARMED - SEL=takeoff" : m, now);
  } else if (c.airborne) {
    c.cmdLand();
    toast("LANDING", now);
  } else {
    c.cmdDisarm(R_USER);
    toast("DISARMED", now);
  }
}

void UiMenu::homeButton(uint8_t btn, uint8_t evt) {
  if (btn == BTN_BACK && evt == EV_LONG) { app->core.cmdKill(); toast("KILL - motors off", now); return; }
  if (btn == BTN_SELECT && evt == EV_LONG) { armToggle(); return; }
  if (evt == EV_LONG) return;
  if (btn == BTN_UP) homePage = (uint8_t)((homePage + HOME_PAGES - 1) % HOME_PAGES);
  if (btn == BTN_DOWN) homePage = (uint8_t)((homePage + 1) % HOME_PAGES);
  if (btn == BTN_SELECT && evt == EV_SHORT) {
    // when armed and waiting on the ground, SELECT = take off (a quick way to fly with the buttons)
    if (app->core.state == ST_IDLE) { app->core.cmdTakeoff(now); toast("TAKEOFF", now); return; }
    screen = SC_MENU;
  }
}

void UiMenu::listButton(uint8_t btn, uint8_t evt, int count) {
  (void)evt;
  if (btn == BTN_UP) sel = (int8_t)((sel + count - 1) % count);
  if (btn == BTN_DOWN) sel = (int8_t)((sel + 1) % count);
  if (sel < top) top = sel;
  if (sel >= top + ROWS) top = (int8_t)(sel - ROWS + 1);
}

void UiMenu::menuButton(uint8_t btn, uint8_t evt) {
  if (btn == BTN_BACK) { screen = SC_HOME; return; }
  if (btn == BTN_UP || btn == BTN_DOWN) {
    sel = menuSel; top = menuTop;
    listButton(btn, evt, MENU_COUNT);
    menuSel = sel; menuTop = top;
    return;
  }
  if (btn == BTN_SELECT && evt == EV_SHORT) {
    sel = 0; top = 0; editing = false;
    if (menuSel == 0) screen = SC_FLIGHT;
    else if (menuSel == 1) screen = SC_CAL;
    else if (menuSel == 2) screen = SC_STORAGE;
    else if (menuSel < MENU_FIXED_TOP + PG_COUNT) {
      group = (uint8_t)(menuSel - MENU_FIXED_TOP);
      buildParamList();
      screen = SC_PARAMS;
    } else screen = SC_ABOUT;
  }
}

static const char *const kFlightItems[] = { "Arm/Disarm (hold)", "Takeoff", "Land", "KILL motors (hold)", "Takeoff height" };

void UiMenu::flightButton(uint8_t btn, uint8_t evt) {
  FlightCore &c = app->core;
  if (editing) {   // editing the take-off height
    int idx = 0;
    while (idx < paramsCount() - 1 && paramDef(idx).offset != offsetof(Params, takeoffAlt)) idx++;
    const ParamDef &d = paramDef(idx);
    if (btn == BTN_UP || btn == BTN_DOWN) {
      const float k = (repeatCount > 10) ? 5.0f : 1.0f;
      paramSet(app->params, idx, paramGet(app->params, idx) + (btn == BTN_UP ? 1.0f : -1.0f) * d.step * k);
    } else editing = false;
    return;
  }
  if (btn == BTN_BACK && evt == EV_SHORT) { screen = SC_MENU; return; }
  if (btn == BTN_UP || btn == BTN_DOWN) { listButton(btn, evt, 5); return; }
  if (btn != BTN_SELECT) return;
  switch (sel) {
    case 0: if (evt == EV_LONG) armToggle(); else toast("Hold SELECT to arm", now); break;
    case 1: if (evt == EV_SHORT) { const int8_t r = c.cmdTakeoff(now); toast(r == ARM_OK ? "TAKEOFF" : armResultName(r), now); } break;
    case 2: if (evt == EV_SHORT) { c.cmdLand(); toast("LAND", now); } break;
    case 3: if (evt == EV_LONG) { c.cmdKill(); toast("KILL - motors off", now); } else toast("Hold SELECT to kill", now); break;
    case 4: if (evt == EV_SHORT) editing = true; break;
  }
}

void UiMenu::buildParamList() {
  paramN = 0;
  for (int i = 0; i < paramsCount() && paramN < (int)sizeof(paramIdx); i++)
    if (paramDef(i).group == group) paramIdx[paramN++] = (int8_t)i;
}

void UiMenu::paramsButton(uint8_t btn, uint8_t evt) {
  if (paramN == 0) { screen = SC_MENU; return; }
  const int idx = paramIdx[sel];
  const ParamDef &d = paramDef(idx);
  if (editing) {
    if (btn == BTN_UP || btn == BTN_DOWN) {
      float k = 1.0f;
      if (repeatCount > 30) k = 25.0f; else if (repeatCount > 10) k = 5.0f;
      float v = paramGet(app->params, idx) + (btn == BTN_UP ? 1.0f : -1.0f) * d.step * k;
      if (d.kind == PK_ENUM) {   // wrap around
        if (v > d.maxV) v = d.minV;
        if (v < d.minV) v = d.maxV;
      }
      paramSet(app->params, idx, v);
    } else {
      editing = false;
    }
    return;
  }
  if (btn == BTN_BACK) { screen = SC_MENU; return; }
  if (btn == BTN_UP || btn == BTN_DOWN) { listButton(btn, evt, paramN); return; }
  if (btn == BTN_SELECT && evt == EV_SHORT) {
    if (d.kind == PK_BOOL) {
      paramSet(app->params, idx, paramGet(app->params, idx) > 0.5f ? 0.0f : 1.0f);
      if (d.offset == offsetof(Params, compassOn)) app->core.ekf.resetMagHealth();
    } else {
      editing = true;
    }
  }
}

static const char *const kCalItems[] = { "Gyro (keep still)", "Level (keep level)", "Compass (yaw 360)", "Baro zero", "Reset estimator" };

void UiMenu::calButton(uint8_t btn, uint8_t evt) {
  FlightCore &c = app->core;
  if (btn == BTN_BACK) {
    if (c.sens.calibrating()) { c.sens.abortCalibration(); toast("Calibration aborted", now); }
    else screen = SC_MENU;
    return;
  }
  if (c.sens.calibrating() == SensorProcessor::CAL_MAG && btn == BTN_SELECT && evt == EV_SHORT) {
    toast(c.finishCompassCalibration() ? "Compass OK - Save!" : "Compass FAILED", now);
    return;
  }
  if (c.sens.calibrating()) return;
  if (btn == BTN_UP || btn == BTN_DOWN) { listButton(btn, evt, 5); return; }
  if (btn != BTN_SELECT || evt != EV_SHORT) return;
  if (sel == 4) {
    if (c.armed()) toast("Disarm first", now);
    else { c.resetEstimator(); toast("Estimator reset", now); }
    return;
  }
  static const uint8_t kinds[4] = { SensorProcessor::CAL_GYRO, SensorProcessor::CAL_LEVEL, SensorProcessor::CAL_MAG,
                                    SensorProcessor::CAL_BARO };
  if (!c.startCalibration(kinds[sel])) toast("Disarm first", now);
}

static const char *const kStorageItems[] = { "Save settings", "Load saved", "Factory defaults (hold)" };

void UiMenu::storageButton(uint8_t btn, uint8_t evt) {
  if (btn == BTN_BACK) { screen = SC_MENU; return; }
  if (btn == BTN_UP || btn == BTN_DOWN) { listButton(btn, evt, 3); return; }
  if (btn != BTN_SELECT) return;
  if (app->core.armed()) { toast("Disarm first", now); return; }
  if (sel == 0 && evt == EV_SHORT) { app->saveRequested = true; toast("Saving...", now); }
  if (sel == 1 && evt == EV_SHORT) { app->loadRequested = true; toast("Loading...", now); }
  if (sel == 2) {
    if (evt == EV_LONG) { app->restoreDefaults(); toast("Defaults (not saved)", now); }
    else toast("Hold SELECT", now);
  }
}

// ------------------------------------------------------------------ drawing helpers
void UiMenu::drawTitle(Gfx &g, const char *t, int page, int pages) {
  g.fillRect(0, 0, GFX_W, 8);
  g.text(1, 0, t, false);
  if (pages > 1) {
    char b[8], n[4];
    fmtInt(b, page + 1);
    strcat(b, "/");
    strcat(b, fmtInt(n, pages));
    g.text(GFX_W - 1 - 6 * (int)strlen(b), 0, b, false);
  }
}

void UiMenu::kv(Gfx &g, int x, int y, const char *k, float v, int dec, const char *unit) {
  char b[16];
  int px = g.text(x, y, k);
  px = g.text(px, y, fmtFloat(b, v, dec));
  if (unit) g.text(px, y, unit);
}

void UiMenu::drawList(Gfx &g, const char *title, const char *const *items, int count, int selected, int topRow) {
  drawTitle(g, title, 0, 1);
  for (int r = 0; r < ROWS && topRow + r < count; r++) {
    const int y = 8 + 8 * r;
    g.text(2, y + 1, items[topRow + r]);
    if (topRow + r == selected) g.invertRect(0, y, GFX_W, 8);
  }
}

// ------------------------------------------------------------------ screens
void UiMenu::drawHome(Gfx &g) {
  FlightCore &c = app->core;
  char b[24];
  static const char *const titles[HOME_PAGES] = { "STATUS", "ATTITUDE", "ALTITUDE", "MOTORS", "RC INPUT", "SENSORS", "SYSTEM" };
  drawTitle(g, titles[homePage], homePage, HOME_PAGES);
  const float roll = c.ekf.roll() * FC_RAD2DEG, pitch = c.ekf.pitch() * FC_RAD2DEG;
  float yaw = c.ekf.yaw() * FC_RAD2DEG;
  if (yaw < 0.0f) yaw += 360.0f;

  switch (homePage) {
    case 0: {
      g.text(0, 9, sessionName(*app, now));
      strcpy(b, stateName(c.state));
      if (c.armed()) strcat(b, c.mode == MODE_MANUAL ? " MANUAL" : " ALTHOLD");
      g.text(0, 18, b);
      kv(g, 0, 27, "Alt ", c.altf.alt, 2, "m");
      kv(g, 72, 27, "Vz ", c.altf.vz, 1);
      kv(g, 0, 36, "R ", roll, 1);
      kv(g, 64, 36, "P ", pitch, 1);
      kv(g, 0, 45, "Hdg ", yaw, 0);
      g.text(64, 45, (c.ekf.magHealthy && app->params.compassOn > 0.5f) ? "CMP ok" : "CMP off");
      strcpy(b, "Last: ");
      strcat(b, reasonName(c.reason));
      if (c.crashed) strcpy(b, "!! CRASH detected !!");
      g.text(0, 55, b);
      break;
    }
    case 1: {   // artificial horizon (left) + numbers (right)
      const int cx = 31, cy = 36, rad = 26;
      g.rect(cx - rad - 1, cy - rad - 1, 2 * rad + 3, 2 * rad + 3);
      const float r = c.ekf.roll(), p = c.ekf.pitch();
      const float off = p * FC_RAD2DEG * 0.8f;   // pixels per degree of pitch
      const float dx = cosf(r) * rad, dy = sinf(r) * rad;
      // the horizon tilts opposite to the roll: right side down -> horizon goes up on the right
      const int y0 = cy + (int)(off + dy), y1 = cy + (int)(off - dy);
      g.line(cx - (int)dx, y0, cx + (int)dx, y1);
      g.hline(cx - 8, cy, 5); g.hline(cx + 4, cy, 5); g.pixel(cx, cy);   // aircraft symbol
      kv(g, 66, 10, "R ", roll, 1);
      kv(g, 66, 19, "P ", pitch, 1);
      kv(g, 66, 28, "Y ", yaw, 0);
      kv(g, 66, 37, "rSp ", c.rollSp * FC_RAD2DEG, 0);
      kv(g, 66, 46, "pSp ", c.pitchSp * FC_RAD2DEG, 0);
      g.text(66, 55, c.att.headingLocked ? "HDG LOCK" : "HDG free");
      break;
    }
    case 2: {
      kv(g, 0, 9, "Alt ", c.altf.alt, 2, "m");
      kv(g, 70, 9, "Tgt ", c.altTarget, 1);
      kv(g, 0, 18, "Climb ", c.altf.vz, 2, "m/s");
      kv(g, 0, 27, "Baro ", c.sf.baroAlt, 2, "m");
      // mini graph: last 64 samples, 0..4 m scaled to 26 px
      g.rect(0, 36, 130 - 2, 28);
      for (int i = 0; i < 63; i++) {
        const int a = altHist[(histIdx + i) & 63], b2 = altHist[(histIdx + i + 1) & 63];
        g.line(1 + 2 * i, 62 - a, 3 + 2 * i, 62 - b2);
      }
      break;
    }
    case 3: {
      static const char *const lab[4] = { "FR", "RL", "FL", "RR" };
      for (int i = 0; i < 4; i++) {
        const int x = 4 + 32 * i;
        const float m = c.motor[i];
        g.rect(x, 10, 14, 38);
        const int h = (int)(m * 36.0f + 0.5f);
        g.fillRect(x + 1, 47 - h, 12, h);
        g.text(x + 1, 49, lab[i]);
        g.text(x - 2, 57, fmtInt(b, (long)(m * 100.0f + 0.5f)));
        g.text(x - 2 + 6 * (int)strlen(b), 57, "%");
      }
      break;
    }
    case 4: {   // two stick boxes + numbers
      const PilotState &ps = c.pilot;
      float thrStick = ps.manual ? (ps.manualCmd * 2.0f - 1.0f) : ps.climb;
      g.rect(0, 10, 30, 30);
      g.rect(34, 10, 30, 30);
      g.fillRect(14 + (int)(ps.yaw * 13.0f), 24 - (int)(thrStick * 13.0f), 3, 3);
      g.fillRect(48 + (int)(ps.roll * 13.0f), 24 + (int)(ps.pitch * 13.0f), 3, 3);
      kv(g, 68, 10, "T ", ps.rawThr, 2);
      kv(g, 68, 19, "R ", ps.rawRoll, 2);
      kv(g, 68, 28, "P ", ps.rawPitch, 2);
      kv(g, 68, 37, "Y ", ps.rawYaw, 2);
      g.text(0, 46, ps.manual ? "Mode: MANUAL thr" : "Mode: ALT HOLD");
      char hz[8];
      strcpy(b, ps.linkLost ? "LINK LOST " : (ps.everReceived ? "link ok " : "no input "));
      strcat(b, fmtInt(hz, ps.rateHz));
      strcat(b, "Hz");
      g.text(0, 55, b);
      break;
    }
    case 5: {
      const SensorFrame &s = c.sf;
      g.text(0, 9, "acc"); kv(g, 24, 9, "", s.acc.x, 1); kv(g, 58, 9, "", s.acc.y, 1); kv(g, 92, 9, "", s.acc.z, 1);
      g.text(0, 18, "gyr");
      kv(g, 24, 18, "", s.gyro.x * FC_RAD2DEG, 0); kv(g, 58, 18, "", s.gyro.y * FC_RAD2DEG, 0); kv(g, 92, 18, "", s.gyro.z * FC_RAD2DEG, 0);
      g.text(0, 27, "raw"); kv(g, 24, 27, "", s.rawAcc[2], 0); kv(g, 70, 27, "", s.rawGyro[2], 0);
      g.text(0, 36, "mag"); kv(g, 24, 36, "", s.mag.x, 0); kv(g, 58, 36, "", s.mag.y, 0); kv(g, 92, 36, "", s.mag.z, 0);
      kv(g, 0, 45, "baro ", s.baroAlt, 2);
      kv(g, 70, 45, "bz ", c.ekf.bias.z * FC_RAD2DEG, 2);
      kv(g, 0, 55, "N ", c.homeNorth, 1); kv(g, 64, 55, "E ", c.homeEast, 1);
      break;
    }
    default: {
      kv(g, 0, 9, "IMU ", c.imuHz, 0, "Hz");
      kv(g, 64, 9, "Ctl ", c.ctrlHz, 0, "Hz");
      kv(g, 0, 18, "Step ", (float)c.loopUs, 0, "us");
      kv(g, 0, 27, "CRC err ", (float)app->parser.crcErrors, 0);
      kv(g, 0, 36, "Rx imu ", (float)app->rxCount[0], 0);
      kv(g, 0, 45, "Tx mot ", (float)app->txActuators, 0);
      kv(g, 0, 55, "Up ", (float)(now / 1000000u), 0, "s");
      g.text(70, 55, app->bootFromDefaults ? "dflt" : "saved");
      break;
    }
  }
}

void UiMenu::drawMenu(Gfx &g) {
  const char *items[MENU_COUNT];
  for (int i = 0; i < MENU_COUNT; i++) items[i] = menuName(i);
  drawList(g, "MENU", items, MENU_COUNT, menuSel, menuTop);
}

void UiMenu::drawFlight(Gfx &g) {
  drawList(g, "FLIGHT", kFlightItems, 5, sel, top);
  char b[12];
  if (top <= 4 && 4 < top + ROWS) {
    const int y = 8 + 8 * (4 - top) + 1;
    fmtFloat(b, app->params.takeoffAlt, 1);
    g.fillRect(96, y - 1, 32, 8, sel == 4);
    g.text(98, y, b, sel != 4);
    if (editing) g.invertRect(96, y - 1, 32, 8);
  }
  g.fillRect(0, 56, GFX_W, 8, false);
  char s[24];
  strcpy(s, stateName(app->core.state));
  strcat(s, " ");
  strcat(s, fmtFloat(b, app->core.altf.alt, 1));
  strcat(s, "m");
  g.text(0, 57, s);
}

void UiMenu::drawParams(Gfx &g) {
  drawTitle(g, paramGroupName(group), 0, 1);
  char b[16];
  for (int r = 0; r < ROWS && top + r < paramN; r++) {
    const int idx = paramIdx[top + r];
    const ParamDef &d = paramDef(idx);
    const float v = paramGet(app->params, idx);
    const int y = 8 + 8 * r;
    g.text(2, y + 1, d.name);
    if (d.kind == PK_BOOL) strcpy(b, v > 0.5f ? "ON" : "off");
    else if (d.kind == PK_ENUM && d.enumNames) strcpy(b, d.enumNames[(int)v]);
    else fmtFloat(b, v, d.decimals);
    const int w = 6 * (int)strlen(b);
    g.text(GFX_W - 2 - w, y + 1, b);
    if (top + r == sel) {
      if (editing) g.invertRect(GFX_W - 4 - w, y, w + 4, 8);
      else g.invertRect(0, y, GFX_W, 8);
    }
  }
  if (group == PG_LOOP && paramN + 1 < ROWS) {   // live loop-rate readout
    const FlightCore &c = app->core;
    kv(g, 2, 26, "IMU  ", c.imuHz, 0, " Hz");
    kv(g, 2, 35, "Ctrl ", c.ctrlHz, 0, " Hz");
    kv(g, 2, 44, "Step ", (float)c.loopUs, 0, " us");
    g.text(2, 55, "SYNC = 1 step/packet");
  }
}

void UiMenu::drawCal(Gfx &g) {
  SensorProcessor &s = app->core.sens;
  if (s.calibrating()) {
    drawTitle(g, "CALIBRATING", 0, 1);
    static const char *const names[] = { "", "Gyro: keep still", "Level: keep level", "Compass: yaw 360", "Baro zero" };
    g.text(0, 14, names[s.calibrating() < 5 ? s.calibrating() : 0]);
    g.bar(4, 28, 120, 10, s.calProgress());
    if (s.calibrating() == SensorProcessor::CAL_MAG) g.text(0, 44, "then press SELECT");
    g.text(0, 55, "BACK = abort");
    return;
  }
  drawList(g, "CALIBRATE", kCalItems, 5, sel, top);
  if (s.lastCalResult) {
    g.fillRect(0, 48, GFX_W, 16, false);
    g.hline(0, 48, GFX_W);
    g.text(0, 52, s.lastCalResult > 0 ? "Last: OK (Save it!)" : "Last: FAILED");
  }
}

void UiMenu::drawStorage(Gfx &g) {
  drawList(g, "STORAGE", kStorageItems, 3, sel, top);
  g.text(0, 40, app->bootFromDefaults ? "Boot: defaults" : "Boot: saved");
  const char *r = "";
  if (app->lastSaveResult == 1) r = "Last save: OK";
  else if (app->lastSaveResult == -1) r = "Last save: FAILED";
  else if (app->lastSaveResult == -2) r = "Save refused: armed";
  g.text(0, 49, r);
  if (app->lastLoadResult == 1) g.text(0, 57, "Load: OK");
  else if (app->lastLoadResult == -1) g.text(0, 57, "Load: none stored");
}

void UiMenu::drawAbout(Gfx &g) {
  char b[12];
  drawTitle(g, "ABOUT", 0, 1);
  g.text(0, 10, FW_VERSION);
  g.text(0, 19, "Arduino UNO R4 Minima");
  g.text(0, 28, "Aerovidya HIL v1");
  kv(g, 0, 37, "Params v", (float)PARAMS_VERSION, 0);
  kv(g, 0, 46, "Settings ", (float)app->settingsSize(), 0, " B");
  strcpy(b, "BACK");
  g.text(0, 55, b);
}

bool UiMenu::render(uint8_t *fb, uint32_t nowUs) {
  now = nowUs;
  // altitude history for the graph (5 Hz)
  if ((uint32_t)(nowUs - lastHistUs) >= 200000u) {
    lastHistUs = nowUs;
    float a = app->core.altf.alt / 4.0f * 26.0f;
    altHist[histIdx] = (int8_t)fcClamp(a, 0.0f, 26.0f);
    histIdx = (uint8_t)((histIdx + 1) & 63);
  }
  if (!dirty && (uint32_t)(nowUs - lastRenderUs) < 200000u) return false;
  dirty = false;
  lastRenderUs = nowUs;
  Gfx g(fb);
  g.clear();
  switch (screen) {
    case SC_HOME: drawHome(g); break;
    case SC_MENU: drawMenu(g); break;
    case SC_FLIGHT: drawFlight(g); break;
    case SC_PARAMS: drawParams(g); break;
    case SC_CAL: drawCal(g); break;
    case SC_STORAGE: drawStorage(g); break;
    default: drawAbout(g); break;
  }
  if (toastUntil && (int32_t)(toastUntil - nowUs) > 0) {
    g.fillRect(0, 24, GFX_W, 13, false);
    g.rect(0, 24, GFX_W, 13);
    g.text(3, 27, toastMsg);
  } else {
    toastUntil = 0;
  }
  return true;
}
