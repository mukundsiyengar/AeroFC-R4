// aerofc_sitl.cpp - "software in the loop": the UNMODIFIED firmware core (firmware/AeroFC_R4/src/core) as a
// Linux program that talks to the organisers' bridge over the virtual serial port, exactly like the Arduino does
// over USB.  Lets you fly (and test) this flight controller in the real Gazebo without the board.
//
//   ./aerofc_sitl                              opens ./fc_fake (the socat end the fake Arduino normally uses)
//   ./aerofc_sitl --port /dev/pts/5            another serial device
//   ./aerofc_sitl --set "Alt limit=1" --set "Max alt=20"     change settings (names as on the OLED)
//   ./aerofc_sitl --settings ~/aerofc_sitl.cfg                keep settings in a file (OLED "Save" = save there)
//   ./aerofc_sitl --list                       print all settings and exit
//
// Built by tools/sitl/build_sitl.py (zig, static x86_64 Linux binary).  Prints one status line per second.
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include "../../firmware/AeroFC_R4/src/core/fc_app.h"

static FcApp app;
static volatile sig_atomic_t quit = 0;
static void onSignal(int) { quit = 1; }

static uint32_t nowUs() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)((uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u);
}

static int findParam(const char *name) {
  for (int i = 0; i < paramsCount(); i++)
    if (strcasecmp(paramDef(i).name, name) == 0) return i;
  return -1;
}

static bool applySet(const char *arg) {
  char name[32];
  const char *eq = strchr(arg, '=');
  if (!eq || eq == arg || (size_t)(eq - arg) >= sizeof(name)) return false;
  memcpy(name, arg, (size_t)(eq - arg));
  name[eq - arg] = 0;
  const int i = findParam(name);
  if (i < 0) return false;
  const ParamDef &d = paramDef(i);
  float v;
  // enums can be given by name: --set "Speed=SPORT"
  if (d.enumNames) {
    v = -1;
    for (int k = 0; k <= (int)d.maxV; k++)
      if (strcasecmp(d.enumNames[k], eq + 1) == 0) v = (float)k;
    if (v < 0) v = strtof(eq + 1, nullptr);
  } else {
    v = strtof(eq + 1, nullptr);
  }
  paramSet(app.params, i, v);
  fprintf(stderr, "[sitl] %s = %g\n", d.name, (double)paramGet(app.params, i));
  return true;
}

static bool loadSettings(const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) return false;
  uint8_t buf[1024];
  const size_t n = fread(buf, 1, sizeof(buf), f);
  fclose(f);
  return app.unpackSettings(buf, n);
}

static bool saveSettings(const char *path) {
  uint8_t buf[1024];
  const size_t n = app.packSettings(buf, sizeof(buf));
  FILE *f = fopen(path, "wb");
  if (!f) return false;
  const bool ok = fwrite(buf, 1, n, f) == n;
  fclose(f);
  return ok;
}

static int openPort(const char *path) {
  const int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;
  termios t;
  if (tcgetattr(fd, &t) == 0) {
    cfmakeraw(&t);
    cfsetispeed(&t, B115200);
    cfsetospeed(&t, B115200);
    tcsetattr(fd, TCSANOW, &t);
  }
  return fd;
}

static void writeAll(int fd, const uint8_t *p, size_t n) {
  while (n > 0 && !quit) {
    const ssize_t w = write(fd, p, n);
    if (w > 0) { p += w; n -= (size_t)w; continue; }
    if (w < 0 && errno != EAGAIN && errno != EINTR) return;
    pollfd pf = { fd, POLLOUT, 0 };
    poll(&pf, 1, 5);
  }
}

int main(int argc, char **argv) {
  const char *port = "fc_fake";
  const char *settings = nullptr;
  app.init();
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--port") && i + 1 < argc) port = argv[++i];
    else if (!strcmp(argv[i], "--settings") && i + 1 < argc) settings = argv[++i];
    else if (!strcmp(argv[i], "--list")) {
      for (int k = 0; k < paramsCount(); k++)
        printf("%-10s = %-8g  %s\n", paramDef(k).name, (double)paramGet(app.params, k), paramDef(k).help);
      return 0;
    }
  }
  if (settings) {
    if (loadSettings(settings)) { app.bootFromDefaults = false; fprintf(stderr, "[sitl] settings loaded from %s\n", settings); }
    else fprintf(stderr, "[sitl] no valid settings in %s: defaults\n", settings);
  }
  for (int i = 1; i < argc; i++)
    if (!strcmp(argv[i], "--set") && i + 1 < argc && !applySet(argv[++i])) {
      fprintf(stderr, "[sitl] unknown setting '%s' (see --list)\n", argv[i]);
      return 2;
    }

  const int fd = openPort(port);
  if (fd < 0) { fprintf(stderr, "[sitl] cannot open %s: %s\n", port, strerror(errno)); return 1; }
  signal(SIGINT, onSignal);
  signal(SIGTERM, onSignal);
  fprintf(stderr, "[sitl] %s on %s - waiting for the bridge's HELLO\n", FW_VERSION, port);

  uint8_t in[512], out[512];
  uint32_t lastPrint = nowUs();
  while (!quit) {
    pollfd pf = { fd, POLLIN, 0 };
    poll(&pf, 1, 1);                               // wake up at least every 1 ms (timers, 50 Hz heartbeat ...)
    const uint32_t now = nowUs();
    for (;;) {
      const ssize_t n = read(fd, in, sizeof(in));
      if (n <= 0) break;
      app.feed(in, (size_t)n, now);
    }
    app.tick(now);
    for (int guard = 0; guard < 16; guard++) {
      const size_t n = app.nextUsbPacket(out, sizeof(out), now);
      if (!n) break;
      writeAll(fd, out, n);
    }
    if (settings && app.saveRequested) {
      app.lastSaveResult = saveSettings(settings) ? 1 : -1;
      app.saveRequested = false;
    }
    if (settings && app.loadRequested) {
      app.lastLoadResult = loadSettings(settings) ? 1 : -1;
      app.loadRequested = false;
    }
    if ((uint32_t)(now - lastPrint) >= 1000000u) {
      lastPrint = now;
      const FlightCore &c = app.core;
      fprintf(stderr, "[sitl] %3.0f Hz %-8s %-8s alt %7.2f m (target %7.2f)  N %7.2f E %7.2f  v %5.2f m/s  pos %-9s %s  "
                      "yaw %4.0f off %5.1f gps %4.2f m %s %s%s\n",
              (double)c.imuHz, app.session == SES_RUNNING ? "RUNNING" : "waiting", stateName(c.state), (double)c.altf.alt,
              (double)c.altTarget, (double)c.pos.kn.p, (double)c.pos.ke.p,
              (double)sqrtf(c.pos.kn.v * c.pos.kn.v + c.pos.ke.v * c.pos.ke.v),
              c.posActive ? posStateName(c.posState) : (c.posFailed ? "FAILED" : "OFF"),
              speedModeName((uint8_t)app.params.speedMode), (double)(c.ekf.yaw() * FC_RAD2DEG),
              (double)(c.aidYawOffset * FC_RAD2DEG), (double)c.pos.gpsNoiseRaw(), c.posWhy,
              c.reason ? "reason " : "", c.reason ? reasonName(c.reason) : "");
    }
  }
  close(fd);
  fprintf(stderr, "[sitl] stopped\n");
  return 0;
}
