// navigator.h - where the drone should go by itself: to a point, along a mission, or home.
//
// Return to home, like a DJI drone:
//   CLIMB    brake, then up to the RTH height if it is lower, to clear what it flew around
//   TURN     nose towards home
//   CRUISE   straight home at the set speed
//   DESCEND  down to the hover height, over home
//   HOVER    stays there until the pilot takes over, a landing is commanded or someone holds the drone
// A trip to a point or along a mission is TURN + CRUISE, changing height on the way.
//
// Landing on a marker, seen by the camera (tools/vision/marker_tracker.py):
//   ALIGN    flies over the marker and steadies
//   DOWN     comes down slowly while it stays lined up; drifting off, it stops coming down until it is lined up again
//   Low down the camera can't see the whole marker any more, so from there it lands on the last place it saw it.
//   Marker lost for 3 s: it has been holding its height meanwhile, and now lands normally.
//
// Everything is in the position filter's frame (north / east from the first GPS fix, in metres).  This file only decides
// where to go: flight_core.cpp flies it, with the position controller.
#pragma once
#include "fc_math.h"
#include "params.h"

enum NavKind : uint8_t { NAV_NONE = 0, NAV_GOTO, NAV_MISSION, NAV_RTH, NAV_MARKER };
enum NavStage : uint8_t { STAGE_NONE = 0, STAGE_CLIMB, STAGE_TURN, STAGE_CRUISE, STAGE_DESCEND, STAGE_HOVER, STAGE_ALIGN, STAGE_DOWN };
enum NavEvent : uint8_t { NAV_EV_NONE = 0, NAV_EV_ARRIVED, NAV_EV_MISSION_DONE, NAV_EV_NO_PROGRESS, NAV_EV_LAND_NOW };

struct NavIn {
  float pN, pE, vN, vE;     // position filter (m, m/s)
  float heading;            // nose direction in the same frame (rad, clockwise from north)
  float height, vz;         // above the take-off point (m, m/s)
  float dt;
};

struct NavOut {
  bool fly = false;         // true: fly to target at speed. false: position hold does the braking and holding
  float targetN = 0, targetE = 0, speed = 0;
  float height = 0;         // wanted height above the take-off point (m)
  float descentLimit = 0;   // m/s, 0 = the speed mode's own limit
  float yawCmd = 0;         // yaw stick -1..1
};

struct Waypoint { float n, e, h; };   // from the take-off point (m)

class Navigator {
 public:
  static const uint8_t MAX_WAYPOINTS = 16;

  void stop() { kind = NAV_NONE; stage = STAGE_NONE; }
  void goTo(float n, float e, float h, const NavIn &now);
  void goHome(float homeN, float homeE, const Params &p, const NavIn &now);
  // a mission keeps its points until it is cleared; returns false if a point is out of range or there is no point yet
  void clearMission() { wpCount = 0; }
  bool setWaypoint(int index, float n, float e, float h);
  bool startMission(float homeN, float homeE, const NavIn &now);
  // the camera sees the marker at this place (position filter's frame); kept until it is seen again
  void setMarker(float n, float e);
  bool markerFresh() const { return markerAge < MARKER_FRESH_S; }
  bool goMarker(const NavIn &now);
  void markerClock(float dt) { markerAge += dt; }
  NavEvent update(const NavIn &in, const Params &p, NavOut &out);

  const char *kindName() const;
  const char *stageName() const;

  uint8_t kind = NAV_NONE, stage = STAGE_NONE;
  float targetN = 0, targetE = 0, targetH = 0;
  float dist = 0;           // to the target (m)
  uint8_t wpIndex = 0, wpCount = 0;
  float hoverT = 0;         // time in the HOVER stage (s)
  float markerAge = 1e6f;   // since the camera last saw the marker (s)
  float markerN = 0, markerE = 0;

 private:
  static constexpr float MARKER_FRESH_S = 0.5f;
  void startLeg(const NavIn &now);
  Waypoint wp_[MAX_WAYPOINTS];
  float homeN_ = 0, homeE_ = 0;
  float bestDist_ = 0, stuckT_ = 0, turnT_ = 0, arriveT_ = 0;
  bool pausedDown_ = false;   // coming down onto the marker has stopped, at targetH
};
