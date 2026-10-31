// ui_menu.h - OLED dashboard + menu driven by 4 buttons (UP, DOWN, SELECT, BACK), with built-in help.
//
// DASHBOARD (8 pages)   UP/DOWN = page, SELECT = menu (armed on the ground: take off),
//                       hold SELECT = arm / disarm (lands if flying), hold BACK = KILL motors
// MENUS                 UP/DOWN = move, SELECT = open / change, BACK = back.
//                       The bottom line always scrolls an explanation of the selected line.
//                       Hold BACK = full help page for the selected line (while ARMED hold BACK = KILL, everywhere)
// Draws into a 1 KB framebuffer; the Arduino sends it to the SSD1306, the simulator shows it on screen.
#pragma once
#include "fc_app.h"
#include "gfx.h"

enum UiButton : uint8_t { BTN_UP = 0, BTN_DOWN = 1, BTN_SELECT = 2, BTN_BACK = 3 };
enum UiEvent : uint8_t { EV_SHORT = 1, EV_LONG = 2, EV_REPEAT = 3 };

// filled in by the platform (Arduino sketch / simulator) for the SYSTEM page
struct UiPlatform {
  const char *resetCause = "-";
  uint16_t i2cErrors = 0, oledReinits = 0;
  bool watchdogOn = false;
};

class UiMenu {
 public:
  void init(FcApp *a, const UiPlatform *plat = 0);
  void onButton(uint8_t btn, uint8_t evt, uint32_t nowUs);
  // Renders when something changed or periodically.  Returns true if fb was redrawn.
  bool render(uint8_t *fb, uint32_t nowUs);
  void toast(const char *msg, uint32_t nowUs, uint32_t ms = 2000);
  void bootMessage(const char *msg, uint32_t nowUs) { toast(msg, nowUs, 3000); }
  // for the automated tests: every help text and its title, to check that it fits on the screen
  static int helpCount();
  static const char *helpTitle(int i);
  static const char *helpTextAt(int i);

 private:
  enum Screen : uint8_t { SC_HOME = 0, SC_MENU, SC_FLIGHT, SC_PARAMS, SC_CAL, SC_STORAGE, SC_ABOUT, SC_HELPMENU,
                          SC_HELPTEXT };
  FcApp *app = 0;
  const UiPlatform *plat = 0;
  uint8_t screen = SC_HOME, homePage = 0;
  int8_t menuSel = 0, menuTop = 0, sel = 0, top = 0;
  uint8_t group = 0;
  bool editing = false, dirty = true;
  uint16_t repeatCount = 0;
  char toastMsg[22];
  uint32_t toastUntil = 0, lastRenderUs = 0, lastHistUs = 0, now = 0, tickerStartUs = 0;
  int8_t altHist[64];
  uint8_t histIdx = 0;
  // the PLOTS page: 11.6 s of history, one column per 100 ms
  static const int PLOT_N = 116;
  int8_t plotRoll[PLOT_N], plotPitch[PLOT_N], plotYaw[PLOT_N], plotThr[PLOT_N];   // sticks, -100..100 %
  int8_t plotAttR[PLOT_N], plotAttP[PLOT_N];                                      // attitude, degrees x 3
  int16_t plotAlt[PLOT_N];                                                        // height, cm
  uint8_t plotIdx = 0;
  uint32_t lastPlotUs = 0;
  void samplePlots();
  void drawPlots(Gfx &g);
  int8_t paramIdx[24];
  uint8_t paramN = 0;
  // full-screen help
  const char *helpTitleStr = "", *helpText = "";
  uint8_t helpScroll = 0, helpReturn = SC_MENU;

  void buildParamList();
  void homeButton(uint8_t btn, uint8_t evt);
  void menuButton(uint8_t btn, uint8_t evt);
  void listButton(uint8_t btn, int count);
  void flightButton(uint8_t btn, uint8_t evt);
  void paramsButton(uint8_t btn, uint8_t evt);
  void calButton(uint8_t btn, uint8_t evt);
  void storageButton(uint8_t btn, uint8_t evt);
  void helpMenuButton(uint8_t btn, uint8_t evt);
  void armToggle();
  bool editParam(int idx, uint8_t btn);   // UP/DOWN change while editing
  void openHelp(const char *title, const char *text);
  const char *selectedHelp(const char **title);

  void drawTitle(Gfx &g, const char *t, int page, int pages);
  void drawRows(Gfx &g, int count, int selected, int topRow, const char *(*label)(UiMenu *, int, char *),
                const char *(*value)(UiMenu *, int, char *));
  void drawTicker(Gfx &g, const char *text);
  void drawHome(Gfx &g);
  void drawHelpText(Gfx &g);
  void drawCal(Gfx &g);
  void drawAbout(Gfx &g);
  void kv(Gfx &g, int x, int y, const char *k, float v, int dec, const char *unit = 0);

  friend const char *menuLabel(UiMenu *, int, char *);
  friend const char *flightLabel(UiMenu *, int, char *);
  friend const char *flightValue(UiMenu *, int, char *);
  friend const char *paramLabel(UiMenu *, int, char *);
  friend const char *paramValue(UiMenu *, int, char *);
};
