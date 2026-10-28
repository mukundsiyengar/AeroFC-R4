// ui_menu.h - OLED dashboard + menu driven by 4 buttons (UP, DOWN, SELECT, BACK).
//
// HOME (dashboard pages)          UP/DOWN = next page, SELECT = menu,
//                                  hold SELECT = arm / disarm (lands if flying), hold BACK = KILL motors
// MENU                             UP/DOWN = move, SELECT = open, BACK = return
// parameter lists                  SELECT = edit (UP/DOWN change, hold = faster), SELECT/BACK = done
// Draws into a 1 KB framebuffer; the Arduino sends it to the SSD1306, the simulator shows it on screen.
#pragma once
#include "fc_app.h"
#include "gfx.h"

enum UiButton : uint8_t { BTN_UP = 0, BTN_DOWN = 1, BTN_SELECT = 2, BTN_BACK = 3 };
enum UiEvent : uint8_t { EV_SHORT = 1, EV_LONG = 2, EV_REPEAT = 3 };

class UiMenu {
 public:
  void init(FcApp *a);
  void onButton(uint8_t btn, uint8_t evt, uint32_t nowUs);
  // Renders when something changed or every 200 ms.  Returns true if fb was redrawn.
  bool render(uint8_t *fb, uint32_t nowUs);
  void toast(const char *msg, uint32_t nowUs, uint32_t ms = 2000);
  void bootMessage(const char *msg, uint32_t nowUs) { toast(msg, nowUs, 3000); }

 private:
  enum Screen : uint8_t { SC_HOME = 0, SC_MENU, SC_FLIGHT, SC_PARAMS, SC_CAL, SC_STORAGE, SC_ABOUT };
  FcApp *app = 0;
  uint8_t screen = SC_HOME, homePage = 0;
  int8_t menuSel = 0, menuTop = 0, sel = 0, top = 0;
  uint8_t group = 0;
  bool editing = false, dirty = true;
  uint16_t repeatCount = 0;
  char toastMsg[22];
  uint32_t toastUntil = 0, lastRenderUs = 0, lastHistUs = 0, now = 0;
  int8_t altHist[64];
  uint8_t histIdx = 0;
  int8_t paramIdx[24];
  uint8_t paramN = 0;

  void buildParamList();
  void homeButton(uint8_t btn, uint8_t evt);
  void menuButton(uint8_t btn, uint8_t evt);
  void listButton(uint8_t btn, uint8_t evt, int count);
  void flightButton(uint8_t btn, uint8_t evt);
  void paramsButton(uint8_t btn, uint8_t evt);
  void calButton(uint8_t btn, uint8_t evt);
  void storageButton(uint8_t btn, uint8_t evt);
  void armToggle();

  void drawTitle(Gfx &g, const char *t, int page, int pages);
  void drawList(Gfx &g, const char *title, const char *const *items, int count, int selected, int topRow);
  void drawHome(Gfx &g);
  void drawMenu(Gfx &g);
  void drawFlight(Gfx &g);
  void drawParams(Gfx &g);
  void drawCal(Gfx &g);
  void drawStorage(Gfx &g);
  void drawAbout(Gfx &g);
  void kv(Gfx &g, int x, int y, const char *k, float v, int dec, const char *unit = 0);
};
