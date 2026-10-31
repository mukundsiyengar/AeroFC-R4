#include "ui_menu.h"
#include <string.h>
#include <stddef.h>

static const int ROWS = 6;                 // list rows under the title bar (8 px each); the bottom line is the help ticker
static const int COLS = 21;                // characters per line
static const int HOME_PAGES = 9;
static const int MENU_FIXED = 4;           // Flight, Calibrate, Storage, Help, then the parameter groups, then About
static const int MENU_COUNT = MENU_FIXED + PG_COUNT + 1;

// ------------------------------------------------------------------ texts
static const char *const kMenuHelp[MENU_FIXED] = {
  "Arm, take off, land, KILL, speed mode, position hold, height limit.",
  "Gyro, level, compass and barometer calibration. Save afterwards.",
  "Save the settings to flash, load them, or go back to factory defaults.",
  "Explanations of the buttons, every dashboard page and every flight feature.",
};
static const char *const kAboutHelp = "Firmware version, parameter version and size of the saved settings.";

enum FlightAction : int8_t { FA_ARM = 0, FA_TAKEOFF, FA_LAND, FA_KILL, FA_PARAM };
struct FlightItem { const char *label; int8_t action; uint16_t off; const char *help; };
static const FlightItem kFlight[] = {
  { "Arm/Disarm (hold)", FA_ARM, 0, "Hold SELECT to arm (motors idle) or disarm. While flying it lands instead." },
  { "Take off", FA_TAKEOFF, 0, "Arm if needed and climb to TO alt, then hover and hold the position." },
  { "Land", FA_LAND, 0, "Descend and land on the spot; the motors stop by themselves on the ground." },
  { 0, FA_PARAM, (uint16_t)offsetof(Params, speedMode), 0 },
  { 0, FA_PARAM, (uint16_t)offsetof(Params, posHoldOn), 0 },
  { 0, FA_PARAM, (uint16_t)offsetof(Params, posStick), 0 },
  { 0, FA_PARAM, (uint16_t)offsetof(Params, altLimitOn), 0 },
  { 0, FA_PARAM, (uint16_t)offsetof(Params, maxAlt), 0 },
  { 0, FA_PARAM, (uint16_t)offsetof(Params, takeoffAlt), 0 },
  { "KILL motors (hold)", FA_KILL, 0, "Emergency: hold SELECT and the motors stop at once. The drone falls!" },
};
static const int FLIGHT_COUNT = (int)(sizeof(kFlight) / sizeof(kFlight[0]));

struct TextItem { const char *label; const char *help; };
static const TextItem kCal[] = {
  { "Gyro (keep still)", "Measures the gyro offset. Keep the drone still for 2 s. Disarmed only." },
  { "Level (keep level)", "Stores what level is. Keep the drone level and still for 2 s. Disarmed only." },
  { "Compass (yaw 360)", "Start, turn the drone one full circle (yaw), then press SELECT. Works in flight." },
  { "Baro zero", "Sets the current barometer height as zero. Disarmed only." },
  { "Reset estimator", "Restarts the attitude filter (EKF). Disarmed only." },
};
static const TextItem kStorage[] = {
  { "Save settings", "Writes all settings and calibrations to flash, kept after power off. Not while armed." },
  { "Load saved", "Loads the last saved settings again (undo changes)." },
  { "Factory defaults", "Hold SELECT: all settings back to default. Save afterwards to keep them." },
};
static const TextItem kTopics[] = {
  { "Buttons", "UP/DOWN move or change a value. SELECT opens or confirms. BACK goes back. Dashboard: hold SELECT = "
               "arm/disarm, hold BACK = KILL. Menus: hold BACK = help for the selected line. While armed hold BACK "
               "= KILL on every screen." },
  { "Page STATUS", "PC link (WAIT PC / RUNNING / LOST), flight state and mode, speed mode, height, climb rate, "
                   "roll/pitch, heading, compass, position hold and the last disarm or failsafe reason." },
  { "Page ATTITUDE", "Artificial horizon with roll, pitch and heading, the angles the controller asks for (rSp, "
                     "pSp) and whether the heading is locked." },
  { "Page ALTITUDE", "Height, target height, climb rate, barometer and a graph of the last 13 s. CEIL shows the "
                     "height limit when it is on." },
  { "Page POSITION", "GPS position hold: state (HOLD, BRAKE, MOVE, OFF, NOT READY), distance from the hold point, "
                     "speed, position from home, learned wind push and stick style." },
  { "Page MOTORS", "The 4 motor commands seen from above, front at the top, with spin direction (cw/ccw). About 66% "
                   "hovers at any height: the weight stays the same. PWR = power above or below hover, while the speed up or "
                   "down is changing. The bars differ when the drone tilts or turns." },
  { "Page RC INPUT", "The pilot sticks received from the PC, the mode and the packet rate. LINK LOST = no pilot "
                     "packets." },
  { "Page PLOTS", "Live graphs of the last 12 s, on the board itself. RP: roll stick, pitch stick dotted. YT: yaw "
                  "stick, throttle stick dotted. AT: roll, pitch dotted, top and bottom = 40 deg. AL: height, scaled to "
                  "fit, with the height now in m." },
  { "Page SENSORS", "Raw accelerometer, gyro, compass and barometer, the gyro bias the EKF learned, and the GPS "
                    "position from home." },
  { "Page SYSTEM", "Sensor and control rates, time of one control step, CRC errors, packets, uptime, last reset "
                   "cause, OLED I2C errors and whether the watchdog runs." },
  { "Bridge too slow", "The bridge sends fewer than 9 sensor packets a second, so the drone can't be flown and won't "
                       "take off. The bridge is in its slow mode: it says Subprocess Fallback. Fix on that PC: sudo apt "
                       "install gz-harmonic" },
  { "Speed modes", "ECO: 2 m/s, 15 deg, gentle. NORMAL: 5 m/s, 25 deg. SPORT: 10 m/s, 35 deg and faster climb. "
                   "Change in Flight menu, keys 1/2/3 or the X button in the simulator." },
  { "Position hold", "Uses GPS and compass. Let go of the sticks: the drone brakes, then holds its spot and fights "
                     "the wind. SPEED sticks set the speed, ANGLE sticks set the tilt." },
  { "Height & limit", "Throttle centred holds the height (Alt hold). Alt limit ON: the drone never climbs above "
                      "Max alt over the take-off point. OFF: no limit." },
  { "Failsafe", "Pilot link lost for Link t/o seconds: LAND lands on the spot, HOLD hovers in place, NONE does "
                "nothing. Crash: motors stop when tipped over Crash deg." },
  { "Calibration", "Gyro and level: on a flat, still surface. Compass: start, turn one full circle, press SELECT. "
                   "Always Save afterwards." },
  { "Saving", "Changes work at once but are lost at power off unless you use Storage > Save. Hold BACK while "
              "plugging in = factory defaults." },
};
static const int TOPIC_COUNT = (int)(sizeof(kTopics) / sizeof(kTopics[0]));

// all help texts (used by the automated test that checks they display correctly)
int UiMenu::helpCount() { return TOPIC_COUNT + paramsCount() + PG_COUNT + MENU_FIXED + 1 + FLIGHT_COUNT + 5 + 3; }
const char *UiMenu::helpTitle(int i) {
  if (i < TOPIC_COUNT) return kTopics[i].label;
  i -= TOPIC_COUNT;
  if (i < paramsCount()) return paramDef(i).name;
  i -= paramsCount();
  if (i < PG_COUNT) return paramGroupName((uint8_t)i);
  i -= PG_COUNT;
  if (i < MENU_FIXED) return "menu";
  i -= MENU_FIXED;
  if (i < 1) return "About";
  i -= 1;
  if (i < FLIGHT_COUNT) return kFlight[i].label ? kFlight[i].label : "flight";
  i -= FLIGHT_COUNT;
  if (i < 5) return kCal[i].label;
  return kStorage[i - 5].label;
}
const char *UiMenu::helpTextAt(int i) {
  if (i < TOPIC_COUNT) return kTopics[i].help;
  i -= TOPIC_COUNT;
  if (i < paramsCount()) return paramDef(i).help;
  i -= paramsCount();
  if (i < PG_COUNT) return paramGroupHelp((uint8_t)i);
  i -= PG_COUNT;
  if (i < MENU_FIXED) return kMenuHelp[i];
  i -= MENU_FIXED;
  if (i < 1) return kAboutHelp;
  i -= 1;
  if (i < FLIGHT_COUNT) return kFlight[i].help ? kFlight[i].help : paramDef(paramIndexOf(kFlight[i].off)).help;
  i -= FLIGHT_COUNT;
  if (i < 5) return kCal[i].help;
  return kStorage[i - 5].help;
}

// Word wrap: line number `want` of `t` with at most COLS characters (breaks at spaces).  false = no such line.
static bool wrapLine(const char *t, int want, char *out) {
  int line = 0;
  const char *p = t;
  while (*p) {
    while (*p == ' ') p++;
    if (!*p) break;
    const char *start = p, *q = p, *lastBreak = 0;
    while (*q && (q - start) < COLS) { if (*q == ' ') lastBreak = q; q++; }
    const char *end = (!*q || *q == ' ') ? q : (lastBreak ? lastBreak : q);
    if (line == want) {
      const int n = (int)(end - start);
      memcpy(out, start, (size_t)n);
      out[n] = 0;
      return true;
    }
    line++;
    p = end;
  }
  out[0] = 0;
  return false;
}
static int wrapCount(const char *t) {
  char tmp[COLS + 1];
  int n = 0;
  while (wrapLine(t, n, tmp)) n++;
  return n;
}

static const char *sessionName(const FcApp &a, uint32_t now) {
  if (a.linkActive(now)) return a.session == SES_RUNNING ? "PC:RUNNING" : "PC:DATA";
  switch (a.session) {
    case SES_HANDSHAKE: return "PC:HANDSHAKE";
    case SES_RUNNING: return "PC:LOST!";
    case SES_STOPPED: return "PC:STOPPED";
    default: return "PC:WAIT PC";
  }
}
static const char *speedShort(const Params &p) {
  static const char *const s[3] = { "ECO", "NRM", "SPT" };
  const int m = (int)(p.speedMode + 0.5f);
  return (m >= 0 && m < 3) ? s[m] : "?";
}

// ------------------------------------------------------------------ list callbacks
const char *menuLabel(UiMenu *, int i, char *) {
  static const char *const fixed[MENU_FIXED] = { "Flight", "Calibrate", "Storage", "Help" };
  if (i < MENU_FIXED) return fixed[i];
  if (i < MENU_FIXED + PG_COUNT) return paramGroupName((uint8_t)(i - MENU_FIXED));
  return "About";
}
static const char *noValue(UiMenu *, int, char *) { return 0; }
static const char *formatParam(const Params &p, int idx, char *b) {
  const ParamDef &d = paramDef(idx);
  const float v = paramGet(p, idx);
  if (d.kind == PK_BOOL) strcpy(b, v > 0.5f ? "ON" : "off");
  else if (d.kind == PK_ENUM && d.enumNames) strcpy(b, d.enumNames[(int)(v + 0.5f)]);
  else fmtFloat(b, v, d.decimals);
  return b;
}
const char *flightLabel(UiMenu *, int i, char *) {
  return kFlight[i].label ? kFlight[i].label : paramDef(paramIndexOf(kFlight[i].off)).name;
}
const char *flightValue(UiMenu *m, int i, char *b) {
  if (kFlight[i].action != FA_PARAM) return 0;
  return formatParam(m->app->params, paramIndexOf(kFlight[i].off), b);
}
const char *paramLabel(UiMenu *m, int i, char *) { return paramDef(m->paramIdx[i]).name; }
const char *paramValue(UiMenu *m, int i, char *b) { return formatParam(m->app->params, m->paramIdx[i], b); }
static const char *calLabel(UiMenu *, int i, char *) { return kCal[i].label; }
static const char *storageLabel(UiMenu *, int i, char *) { return kStorage[i].label; }
static const char *topicLabel(UiMenu *, int i, char *) { return kTopics[i].label; }

// ------------------------------------------------------------------ setup
void UiMenu::init(FcApp *a, const UiPlatform *p) {
  app = a;
  plat = p;
  screen = SC_HOME;
  homePage = 0;
  memset(altHist, 0, sizeof(altHist));
  memset(plotRoll, 0, sizeof(plotRoll));
  memset(plotPitch, 0, sizeof(plotPitch));
  memset(plotYaw, 0, sizeof(plotYaw));
  memset(plotThr, 0, sizeof(plotThr));
  memset(plotAttR, 0, sizeof(plotAttR));
  memset(plotAttP, 0, sizeof(plotAttP));
  memset(plotAlt, 0, sizeof(plotAlt));
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

void UiMenu::openHelp(const char *title, const char *text) {
  helpReturn = screen;
  helpTitleStr = title;
  helpText = text ? text : "";
  helpScroll = 0;
  screen = SC_HELPTEXT;
}

const char *UiMenu::selectedHelp(const char **title) {
  static char nameBuf[12];
  *title = "Help";
  switch (screen) {
    case SC_MENU:
      *title = menuLabel(this, menuSel, nameBuf);
      if (menuSel < MENU_FIXED) return kMenuHelp[menuSel];
      if (menuSel < MENU_FIXED + PG_COUNT) return paramGroupHelp((uint8_t)(menuSel - MENU_FIXED));
      return kAboutHelp;
    case SC_FLIGHT:
      *title = flightLabel(this, sel, nameBuf);
      return kFlight[sel].help ? kFlight[sel].help : paramDef(paramIndexOf(kFlight[sel].off)).help;
    case SC_PARAMS:
      if (!paramN) return "";
      *title = paramDef(paramIdx[sel]).name;
      return paramDef(paramIdx[sel]).help;
    case SC_CAL: *title = kCal[sel].label; return kCal[sel].help;
    case SC_STORAGE: *title = kStorage[sel].label; return kStorage[sel].help;
    case SC_HELPMENU: *title = kTopics[sel].label; return kTopics[sel].help;
    default: return "";
  }
}

// ------------------------------------------------------------------ buttons
void UiMenu::onButton(uint8_t btn, uint8_t evt, uint32_t nowUs) {
  now = nowUs;
  dirty = true;
  if (evt == EV_REPEAT) repeatCount++;
  else repeatCount = 0;
  // safety first: while armed, holding BACK stops the motors on every screen
  if (btn == BTN_BACK && evt == EV_LONG && app->core.armed()) {
    app->core.cmdKill();
    toast("KILL - motors off", now);
    return;
  }
  // hold BACK in a list = full help for the selected line
  if (btn == BTN_BACK && evt == EV_LONG && screen != SC_HOME && screen != SC_HELPTEXT && !editing) {
    const char *title;
    const char *text = selectedHelp(&title);
    if (text && text[0]) openHelp(title, text);
    return;
  }
  switch (screen) {
    case SC_HOME: homeButton(btn, evt); break;
    case SC_MENU: menuButton(btn, evt); break;
    case SC_FLIGHT: flightButton(btn, evt); break;
    case SC_PARAMS: paramsButton(btn, evt); break;
    case SC_CAL: calButton(btn, evt); break;
    case SC_STORAGE: storageButton(btn, evt); break;
    case SC_HELPMENU: helpMenuButton(btn, evt); break;
    case SC_HELPTEXT: {
      const int lines = wrapCount(helpText);
      if (btn == BTN_DOWN && helpScroll + 7 < lines) helpScroll++;
      else if (btn == BTN_UP && helpScroll > 0) helpScroll--;
      else if (btn == BTN_BACK || btn == BTN_SELECT) screen = helpReturn;
      break;
    }
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
    if (app->core.state == ST_IDLE) { app->core.cmdTakeoff(now); toast("TAKEOFF", now); return; }
    screen = SC_MENU;
    tickerStartUs = now;
  }
}

void UiMenu::listButton(uint8_t btn, int count) {
  if (count <= 0) return;
  if (btn == BTN_UP) sel = (int8_t)((sel + count - 1) % count);
  if (btn == BTN_DOWN) sel = (int8_t)((sel + 1) % count);
  if (sel < top) top = sel;
  if (sel >= top + ROWS) top = (int8_t)(sel - ROWS + 1);
  tickerStartUs = now;
}

void UiMenu::menuButton(uint8_t btn, uint8_t evt) {
  if (btn == BTN_BACK) { screen = SC_HOME; return; }
  if (btn == BTN_UP || btn == BTN_DOWN) {
    sel = menuSel; top = menuTop;
    listButton(btn, MENU_COUNT);
    menuSel = sel; menuTop = top;
    return;
  }
  if (btn == BTN_SELECT && evt == EV_SHORT) {
    sel = 0; top = 0; editing = false;
    tickerStartUs = now;
    if (menuSel == 0) screen = SC_FLIGHT;
    else if (menuSel == 1) screen = SC_CAL;
    else if (menuSel == 2) screen = SC_STORAGE;
    else if (menuSel == 3) screen = SC_HELPMENU;
    else if (menuSel < MENU_FIXED + PG_COUNT) {
      group = (uint8_t)(menuSel - MENU_FIXED);
      buildParamList();
      screen = SC_PARAMS;
    } else screen = SC_ABOUT;
  }
}

bool UiMenu::editParam(int idx, uint8_t btn) {
  if (btn != BTN_UP && btn != BTN_DOWN) return false;
  const ParamDef &d = paramDef(idx);
  float k = 1.0f;
  if (repeatCount > 30) k = 25.0f; else if (repeatCount > 10) k = 5.0f;
  float v = paramGet(app->params, idx) + (btn == BTN_UP ? 1.0f : -1.0f) * d.step * k;
  if (d.kind == PK_ENUM) {   // wrap around
    if (v > d.maxV) v = d.minV;
    if (v < d.minV) v = d.maxV;
  }
  paramSet(app->params, idx, v);
  return true;
}

// SELECT on a parameter line: switches / modes change at once, numbers go into edit mode
static bool quickChange(Params &p, int idx) {
  const ParamDef &d = paramDef(idx);
  if (d.kind == PK_BOOL) { paramSet(p, idx, paramGet(p, idx) > 0.5f ? 0.0f : 1.0f); return true; }
  if (d.kind == PK_ENUM) {
    float v = paramGet(p, idx) + 1.0f;
    if (v > d.maxV) v = d.minV;
    paramSet(p, idx, v);
    return true;
  }
  return false;
}

void UiMenu::flightButton(uint8_t btn, uint8_t evt) {
  FlightCore &c = app->core;
  const FlightItem &it = kFlight[sel];
  if (editing) {
    if (!editParam(paramIndexOf(it.off), btn)) editing = false;
    return;
  }
  if (btn == BTN_BACK && evt == EV_SHORT) { screen = SC_MENU; return; }
  if (btn == BTN_UP || btn == BTN_DOWN) { listButton(btn, FLIGHT_COUNT); return; }
  if (btn != BTN_SELECT) return;
  switch (it.action) {
    case FA_ARM: if (evt == EV_LONG) armToggle(); else toast("Hold SELECT to arm", now); break;
    case FA_TAKEOFF: if (evt == EV_SHORT) { const int8_t r = c.cmdTakeoff(now); toast(r == ARM_OK ? "TAKEOFF" : armResultName(r), now); } break;
    case FA_LAND: if (evt == EV_SHORT) { c.cmdLand(); toast("LAND", now); } break;
    case FA_KILL: if (evt == EV_LONG) { c.cmdKill(); toast("KILL - motors off", now); } else toast("Hold SELECT to kill", now); break;
    default:
      if (evt != EV_SHORT) break;
      if (!quickChange(app->params, paramIndexOf(it.off))) editing = true;
      if (it.off == offsetof(Params, compassOn)) c.ekf.resetMagHealth();
      break;
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
  if (editing) {
    if (!editParam(idx, btn)) editing = false;
    return;
  }
  if (btn == BTN_BACK) { screen = SC_MENU; return; }
  if (btn == BTN_UP || btn == BTN_DOWN) { listButton(btn, paramN); return; }
  if (btn == BTN_SELECT && evt == EV_SHORT) {
    if (!quickChange(app->params, idx)) editing = true;
    if (paramDef(idx).offset == offsetof(Params, compassOn)) app->core.ekf.resetMagHealth();
  }
}

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
  if (btn == BTN_UP || btn == BTN_DOWN) { listButton(btn, 5); return; }
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

void UiMenu::storageButton(uint8_t btn, uint8_t evt) {
  if (btn == BTN_BACK) { screen = SC_MENU; return; }
  if (btn == BTN_UP || btn == BTN_DOWN) { listButton(btn, 3); return; }
  if (btn != BTN_SELECT) return;
  if (app->core.armed()) { toast("Disarm first", now); return; }
  if (sel == 0 && evt == EV_SHORT) { app->saveRequested = true; toast("Saving...", now); }
  if (sel == 1 && evt == EV_SHORT) { app->loadRequested = true; toast("Loading...", now); }
  if (sel == 2) {
    if (evt == EV_LONG) { app->restoreDefaults(); toast("Defaults (not saved)", now); }
    else toast("Hold SELECT", now);
  }
}

void UiMenu::helpMenuButton(uint8_t btn, uint8_t evt) {
  if (btn == BTN_BACK) { screen = SC_MENU; return; }
  if (btn == BTN_UP || btn == BTN_DOWN) { listButton(btn, TOPIC_COUNT); return; }
  if (btn == BTN_SELECT && evt == EV_SHORT) openHelp(kTopics[sel].label, kTopics[sel].help);
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

void UiMenu::drawRows(Gfx &g, int count, int selected, int topRow, const char *(*label)(UiMenu *, int, char *),
                      const char *(*value)(UiMenu *, int, char *)) {
  char lb[16], vb[16];
  for (int r = 0; r < ROWS && topRow + r < count; r++) {
    const int i = topRow + r;
    const int y = 8 + 8 * r;
    g.text(2, y, label(this, i, lb));
    const char *v = value(this, i, vb);
    int w = 0;
    if (v) { w = 6 * (int)strlen(v); g.text(GFX_W - 2 - w, y, v); }
    if (i == selected) {
      if (editing && v) g.invertRect(GFX_W - 4 - w, y - 1, w + 4, 8);
      else g.invertRect(0, y - 1, GFX_W, 8);
    }
  }
  if (count > ROWS) {   // scroll position marker
    const int h = 48 * ROWS / count, y = 8 + (48 - h) * topRow / (count - ROWS);
    g.vline(GFX_W - 1, y, h);
  }
}

// bottom line: the help of the selected line, scrolling when it is longer than the screen
void UiMenu::drawTicker(Gfx &g, const char *text) {
  g.hline(0, 55, GFX_W);
  if (!text || !text[0]) return;
  const int len = (int)strlen(text);
  if (len <= COLS) { g.text(0, 57, text); return; }
  const uint32_t el = (uint32_t)(now - tickerStartUs) / 1000u;
  const int steps = el > 800 ? (int)((el - 800) / 125) : 0;
  const int total = len + 5;
  const int off = steps % total;
  char win[COLS + 1];
  for (int k = 0; k < COLS; k++) {
    const int j = (off + k) % total;
    win[k] = j < len ? text[j] : ' ';
  }
  win[COLS] = 0;
  g.text(0, 57, win);
}

void UiMenu::drawHelpText(Gfx &g) {
  drawTitle(g, helpTitleStr, 0, 1);
  char line[COLS + 1];
  const int lines = wrapCount(helpText);
  for (int r = 0; r < 7; r++) {
    if (!wrapLine(helpText, helpScroll + r, line)) break;
    g.text(0, 9 + 8 * r, line);
  }
  if (helpScroll + 7 < lines) g.text(GFX_W - 6, 57, "v");
  if (helpScroll > 0) g.text(GFX_W - 6, 9, "^");
}

// ------------------------------------------------------------------ dashboard
void UiMenu::drawHome(Gfx &g) {
  FlightCore &c = app->core;
  const Params &p = app->params;
  char b[24];
  static const char *const titles[HOME_PAGES] = { "STATUS", "ATTITUDE", "ALTITUDE", "POSITION", "MOTORS",
                                                  "RC INPUT", "SENSORS", "SYSTEM", "PLOTS" };
  drawTitle(g, titles[homePage], homePage, HOME_PAGES);
  const float roll = c.ekf.roll() * FC_RAD2DEG, pitch = c.ekf.pitch() * FC_RAD2DEG;
  float yaw = c.ekf.yaw() * FC_RAD2DEG;
  if (yaw < 0.0f) yaw += 360.0f;
  const float hAgl = c.altf.alt;
  const int altDec = fcAbs(hAgl) >= 100.0f ? 0 : 2;

  switch (homePage) {
    case 8: drawPlots(g); break;
    case 0: {
      g.text(0, 9, sessionName(*app, now));
      g.text(GFX_W - 18, 9, speedShort(p));
      strcpy(b, stateName(c.state));
      if (c.armed()) strcat(b, c.mode == MODE_MANUAL ? " MANUAL" : " ALTHOLD");
      g.text(0, 18, b);
      kv(g, 0, 27, "Alt ", hAgl, altDec, "m");
      kv(g, 76, 27, "Vz ", c.altf.vz, 1);
      kv(g, 0, 36, "R ", roll, 1);
      kv(g, 64, 36, "P ", pitch, 1);
      kv(g, 0, 45, "Hdg ", yaw, 0);
      if (c.posActive) strcpy(b, "POS HOLD");
      else if (p.posHoldOn > 0.5f) { strcpy(b, "POS:"); strncat(b, c.posWhy[0] ? c.posWhy : "ready", 12); }
      else strcpy(b, (c.ekf.magHealthy && p.compassOn > 0.5f) ? "CMP ok" : "CMP off");
      g.text(54, 45, b);
      strcpy(b, "Last: ");
      strcat(b, reasonName(c.reason));
      if (c.crashed) strcpy(b, "!! CRASH detected !!");
      else if (c.pilot.linkLost) strcpy(b, "!! RC LINK LOST !!");
      else if (c.linkTooSlowShown()) strcpy(b, "!! BRIDGE TOO SLOW !!");
      g.text(0, 55, b);
      break;
    }
    case 1: {   // artificial horizon (left) + numbers (right)
      const int cx = 31, cy = 36, rad = 26;
      g.rect(cx - rad - 1, cy - rad - 1, 2 * rad + 3, 2 * rad + 3);
      const float r = c.ekf.roll(), pt = c.ekf.pitch();
      const float off = pt * FC_RAD2DEG * 0.8f;
      const float dx = cosf(r) * rad, dy = sinf(r) * rad;
      const int y0 = cy + (int)(off + dy), y1 = cy + (int)(off - dy);
      g.line(cx - (int)dx, y0, cx + (int)dx, y1);
      g.hline(cx - 9, cy - 1, 6); g.hline(cx + 4, cy - 1, 6); g.pixel(cx, cy - 2);   // aircraft symbol
      kv(g, 66, 10, "R ", roll, 1);
      kv(g, 66, 19, "P ", pitch, 1);
      kv(g, 66, 28, "Y ", yaw, 0);
      kv(g, 66, 37, "rSp ", c.rollSp * FC_RAD2DEG, 0);
      kv(g, 66, 46, "pSp ", c.pitchSp * FC_RAD2DEG, 0);
      g.text(66, 55, c.att.headingLocked ? "HDG LOCK" : "HDG free");
      break;
    }
    case 2: {
      kv(g, 0, 9, "Alt ", hAgl, altDec, "m");
      kv(g, 72, 9, "Tgt ", c.altTarget, altDec > 0 ? 1 : 0);
      kv(g, 0, 18, "Climb ", c.altf.vz, 2, "m/s");
      if (p.altLimitOn > 0.5f) { kv(g, 0, 27, "CEIL ", p.maxAlt, 0, "m"); if (c.atCeiling) g.text(72, 27, "AT CEIL"); }
      else kv(g, 0, 27, "Baro ", c.sf.baroAlt, altDec, "m");
      g.rect(0, 36, GFX_W, 28);
      for (int i = 0; i < 63; i++) {
        const int a = altHist[(histIdx + i) & 63], b2 = altHist[(histIdx + i + 1) & 63];
        g.line(1 + 2 * i, 62 - a, 3 + 2 * i, 62 - b2);
      }
      break;
    }
    case 3: {   // position hold
      if (c.posActive) strcpy(b, posStateName(c.posState));
      else if (p.posHoldOn < 0.5f) strcpy(b, "OFF (switched off)");
      else { strcpy(b, "NOT READY: "); strncat(b, c.posWhy, 10); }
      g.text(0, 9, b);
      kv(g, 0, 18, "Dist ", c.posActive ? c.posCtl.dist : 0.0f, 2, "m");
      const float spd = sqrtf(fcSq(c.pos.kn.v) + fcSq(c.pos.ke.v));
      kv(g, 70, 18, "V ", spd, 1, "m/s");
      kv(g, 0, 27, "N ", c.pos.kn.p, 1);
      kv(g, 64, 27, "E ", c.pos.ke.p, 1);
      kv(g, 0, 36, "Wind ", sqrtf(fcSq(c.posCtl.iN) + fcSq(c.posCtl.iE)), 2, "m/s2");
      strcpy(b, "Stick ");
      strcat(b, p.posStick > 0.5f ? "ANGLE " : "SPEED ");
      strcat(b, speedModeName((uint8_t)(p.speedMode + 0.5f)));
      g.text(0, 45, b);
      kv(g, 0, 55, "GPS sd ", c.pos.sigma() > 9.9f ? 9.9f : c.pos.sigma(), 2, "m");
      break;
    }
    case 4: {   // motors seen from above: front at the top
      // Gazebo / bridge order: 0 FR (ccw), 1 RL (ccw), 2 FL (cw), 3 RR (cw)
      static const uint8_t idx[4] = { 2, 0, 1, 3 };          // FL FR RL RR = screen cells
      static const char *const lab[4] = { "FL cw", "FR ccw", "RL ccw", "RR cw" };
      for (int k = 0; k < 4; k++) {
        const int x = (k & 1) ? 66 : 2, y = (k & 2) ? 37 : 9;
        const float m = c.motor[idx[k]];
        g.text(x, y, lab[k]);
        fmtInt(b, (long)(m * 100.0f + 0.5f));
        strcat(b, "%");
        g.text(x + 60 - 6 * (int)strlen(b), y, b);
        g.bar(x, y + 9, 60, 8, m);
      }
      {
        // the same thrust holds the drone up at any height, so a steady flight always reads about the hover %:
        // what shows the pilot's input is the power above or below hover
        const float hov = thrustToCmd(p, c.altc.integ);
        const long pwr = (long)((c.motor[0] + c.motor[1] + c.motor[2] + c.motor[3]) * 25.0f - hov * 100.0f + 100.5f) - 100;
        strcpy(b, "PWR");
        strcat(b, pwr >= 0 ? "+" : "");
        char num[8];
        fmtInt(num, pwr);
        strcat(b, num);
        strcat(b, "%");
        g.text(0, 28, b);
        g.text(46, 28, "^FRONT");
        strcpy(b, "hov");
        fmtInt(num, (long)(hov * 100.0f + 0.5f));
        strcat(b, num);
        strcat(b, "%");
        g.text(128 - 6 * (int)strlen(b), 28, b);
      }
      break;
    }
    case 5: {   // two stick boxes + numbers
      const PilotState &ps = c.pilot;
      const float thrStick = ps.manual ? (ps.manualCmd * 2.0f - 1.0f) : ps.climb;
      g.rect(0, 10, 30, 30);
      g.rect(34, 10, 30, 30);
      g.fillRect(14 + (int)(ps.yaw * 13.0f), 24 - (int)(thrStick * 13.0f), 3, 3);
      g.fillRect(48 + (int)(ps.roll * 13.0f), 24 + (int)(ps.pitch * 13.0f), 3, 3);
      kv(g, 68, 10, "T ", ps.rawThr, 2);
      kv(g, 68, 19, "R ", ps.rawRoll, 2);
      kv(g, 68, 28, "P ", ps.rawPitch, 2);
      kv(g, 68, 37, "Y ", ps.rawYaw, 2);
      g.text(0, 45, ps.manual ? "Mode: MANUAL thr" : "Mode: ALT HOLD");
      char hz[8];
      strcpy(b, ps.linkLost ? "LINK LOST " : (ps.everReceived ? "link ok " : "no input "));
      strcat(b, fmtInt(hz, ps.rateHz));
      strcat(b, "Hz");
      g.text(0, 55, b);
      break;
    }
    case 6: {
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
      kv(g, 70, 18, "CRC ", (float)app->parser.crcErrors, 0);
      kv(g, 0, 27, "Rx ", (float)app->rxCount[0], 0);
      kv(g, 64, 27, "Tx ", (float)app->txActuators, 0);
      kv(g, 0, 36, "Up ", (float)(now / 1000000u), 0, "s");
      g.text(64, 36, app->bootFromDefaults ? "set:dflt" : "set:saved");
      strcpy(b, "Reset:");
      strncat(b, plat ? plat->resetCause : "-", 12);
      g.text(0, 45, b);
      if (plat) {
        kv(g, 0, 55, "I2C err ", (float)plat->i2cErrors, 0);
        g.text(80, 55, plat->watchdogOn ? "WDT on" : "WDT off");
      }
      break;
    }
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
  drawTitle(g, "CALIBRATE", 0, 1);
  drawRows(g, 5, sel, top, calLabel, noValue);
  if (s.lastCalResult && (uint32_t)(now - tickerStartUs) < 3000000u) {
    g.hline(0, 55, GFX_W);
    g.text(0, 57, s.lastCalResult > 0 ? "Last: OK (Save it!)" : "Last: FAILED");
  } else {
    drawTicker(g, kCal[sel].help);
  }
}

// ------------------------------------------------------------------ live plots
static int8_t toI8(float v) { return (int8_t)fcClamp(v, -127.0f, 127.0f); }

void UiMenu::samplePlots() {
  const FlightCore &c = app->core;
  const PilotState &ps = c.pilot;
  // a command code is not a stick position: the throttle trace shows the stick or the manual throttle (0.66 = centre)
  const float thr = ps.manual ? fcClamp((ps.manualCmd - 0.66f) * 3.0f, -1.0f, 1.0f) : ps.climb;
  plotRoll[plotIdx] = toI8(ps.roll * 100.0f);
  plotPitch[plotIdx] = toI8(ps.pitch * 100.0f);
  plotYaw[plotIdx] = toI8(ps.yaw * 100.0f);
  plotThr[plotIdx] = toI8(thr * 100.0f);
  plotAttR[plotIdx] = toI8(c.ekf.roll() * FC_RAD2DEG * 3.0f);
  plotAttP[plotIdx] = toI8(c.ekf.pitch() * FC_RAD2DEG * 3.0f);
  plotAlt[plotIdx] = (int16_t)fcClamp(c.altf.alt * 100.0f, -32000.0f, 32000.0f);
  plotIdx = (uint8_t)((plotIdx + 1) % PLOT_N);
}

// one band: a line for the first trace, dots on every other column for the second, oldest sample on the left
static void plotBand(Gfx &g, int y0, int h, int8_t fullScale, const int8_t *a, const int8_t *b, int idx, int n, const char *label) {
  const int x0 = 12, mid = y0 + h / 2;
  g.text(0, y0 + (h - 7) / 2, label);
  for (int x = 0; x < n; x += 4) g.pixel(x0 + x, mid);
  const auto Y = [&](int v) {
    const int y = mid - v * (h / 2) / fullScale;
    return y < y0 ? y0 : (y > y0 + h - 1 ? y0 + h - 1 : y);
  };
  for (int i = 0; i + 1 < n; i++) {
    g.line(x0 + i, Y(a[(idx + i) % n]), x0 + i + 1, Y(a[(idx + i + 1) % n]));
    if (!(i & 1)) g.pixel(x0 + i, Y(b[(idx + i) % n]));
  }
}

void UiMenu::drawPlots(Gfx &g) {
  plotBand(g, 9, 12, 100, plotRoll, plotPitch, plotIdx, PLOT_N, "RP");
  plotBand(g, 22, 12, 100, plotYaw, plotThr, plotIdx, PLOT_N, "YT");
  plotBand(g, 35, 12, 120, plotAttR, plotAttP, plotIdx, PLOT_N, "AT");
  // height: scaled to what is in the window, at least 1 m top to bottom
  const int x0 = 12, y0 = 48, h = 16;
  int lo = plotAlt[0], hi = plotAlt[0];
  for (int i = 1; i < PLOT_N; i++) {
    if (plotAlt[i] < lo) lo = plotAlt[i];
    if (plotAlt[i] > hi) hi = plotAlt[i];
  }
  if (hi - lo < 100) hi = lo + 100;
  g.text(0, y0 + 4, "AL");
  int last = 0;
  for (int i = 0; i < PLOT_N; i++) {
    const int v = plotAlt[(plotIdx + i) % PLOT_N];
    const int y = y0 + h - 1 - (v - lo) * (h - 1) / (hi - lo);
    if (i) g.line(x0 + i - 1, last, x0 + i, y);
    last = y;
  }
  kv(g, 80, y0, "", app->core.altf.alt, 1, "m");
}

void UiMenu::drawAbout(Gfx &g) {
  drawTitle(g, "ABOUT", 0, 1);
  g.text(0, 10, FW_VERSION);
  g.text(0, 19, "Arduino UNO R4 Minima");
  g.text(0, 28, "Aerovidya HIL v1");
  kv(g, 0, 37, "Params v", (float)PARAMS_VERSION, 0);
  kv(g, 0, 46, "Settings ", (float)app->settingsSize(), 0, " B");
  g.text(0, 55, "BACK");
}

bool UiMenu::render(uint8_t *fb, uint32_t nowUs) {
  now = nowUs;
  if ((uint32_t)(nowUs - lastHistUs) >= 200000u) {      // altitude history for the graph (5 Hz)
    lastHistUs = nowUs;
    const float a = app->core.altf.alt / 4.0f * 26.0f;
    altHist[histIdx] = (int8_t)fcClamp(a, 0.0f, 26.0f);
    histIdx = (uint8_t)((histIdx + 1) & 63);
  }
  if ((uint32_t)(nowUs - lastPlotUs) >= 100000u) {      // history for the PLOTS page (10 Hz)
    lastPlotUs = nowUs;
    samplePlots();
  }
  // lists redraw 8x per second (the help ticker moves), the dashboard 5x, the plots 10x
  const uint32_t period = screen == SC_HOME && homePage == HOME_PAGES - 1 ? 100000u
                        : (screen == SC_HOME || screen == SC_HELPTEXT) ? 200000u : 125000u;
  if (!dirty && (uint32_t)(nowUs - lastRenderUs) < period) return false;
  dirty = false;
  lastRenderUs = nowUs;
  Gfx g(fb);
  g.clear();
  const char *title;
  switch (screen) {
    case SC_HOME: drawHome(g); break;
    case SC_MENU:
      drawTitle(g, "MENU", 0, 1);
      sel = menuSel; top = menuTop;
      drawRows(g, MENU_COUNT, menuSel, menuTop, menuLabel, noValue);
      drawTicker(g, selectedHelp(&title));
      break;
    case SC_FLIGHT: {
      char t[22];
      strcpy(t, "FLIGHT ");
      strcat(t, stateName(app->core.state));
      drawTitle(g, t, 0, 1);
      drawRows(g, FLIGHT_COUNT, sel, top, flightLabel, flightValue);
      drawTicker(g, selectedHelp(&title));
      break;
    }
    case SC_PARAMS:
      drawTitle(g, paramGroupName(group), 0, 1);
      drawRows(g, paramN, sel, top, paramLabel, paramValue);
      if (group == PG_LOOP && paramN + 3 < ROWS) {   // live loop-rate readout
        const FlightCore &c = app->core;
        kv(g, 2, 24, "IMU  ", c.imuHz, 0, " Hz");
        kv(g, 2, 32, "Ctrl ", c.ctrlHz, 0, " Hz");
        kv(g, 2, 40, "Step ", (float)c.loopUs, 0, " us");
      }
      drawTicker(g, selectedHelp(&title));
      break;
    case SC_CAL: drawCal(g); break;
    case SC_STORAGE: {
      drawTitle(g, "STORAGE", 0, 1);
      drawRows(g, 3, sel, top, storageLabel, noValue);
      g.text(0, 33, app->bootFromDefaults ? "Boot: defaults" : "Boot: saved");
      const char *r = "";
      if (app->lastSaveResult == 1) r = "Last save: OK";
      else if (app->lastSaveResult == -1) r = "Last save: FAILED";
      else if (app->lastSaveResult == -2) r = "Save refused: armed";
      else if (app->lastLoadResult == 1) r = "Load: OK";
      else if (app->lastLoadResult == -1) r = "Load: none stored";
      g.text(0, 42, r);
      drawTicker(g, kStorage[sel].help);
      break;
    }
    case SC_HELPMENU:
      drawTitle(g, "HELP", 0, 1);
      drawRows(g, TOPIC_COUNT, sel, top, topicLabel, noValue);
      drawTicker(g, "SELECT = read, BACK = back");
      break;
    case SC_HELPTEXT: drawHelpText(g); break;
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
