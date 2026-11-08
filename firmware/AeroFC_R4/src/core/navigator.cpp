#include "navigator.h"

static const float TURN_MIN_M = 3.0f;        // closer than this it just goes there, without turning first
static const float TURN_DONE_RAD = 8.0f * FC_DEG2RAD;
static const float TURN_MAX_S = 8.0f;
static const float TURN_BRAKE_SPEED = 1.0f;  // faster than this it stops first: turning fast also stops it learning a wrong heading
static const float YAW_GAIN = 1.5f;          // yaw stick per radian of heading error
static const float ARRIVE_M = 0.15f;         // arrived: this close ...
static const float ARRIVE_SPEED = 0.25f;     // ... this slow ...
static const float ARRIVE_S = 1.0f;          // ... for this long
static const float HEIGHT_OK_M = 0.25f;
static const float PROGRESS_M = 0.5f;        // getting closer means at least this much ...
static const float PROGRESS_S = 5.0f;        // ... in this time
static const float PROGRESS_NEAR_M = 1.0f;   // the last metre is slow on purpose
static const float MARKER_LOST_S = 3.0f;     // not seen for this long: land normally
static const float MARKER_LINED_UP_M = 0.3f; // further off than this, it stops coming down
static const float MARKER_DESCENT = 0.4f;    // m/s
static const float MARKER_COMMIT_M = 1.0f;   // below this the camera can't see the whole marker: land on the last place it saw it

const char *Navigator::kindName() const {
  static const char *const n[] = { "-", "GO TO", "MISSION", "RTH", "MARKER" };
  return kind < 5 ? n[kind] : "?";
}

const char *Navigator::stageName() const {
  static const char *const n[] = { "-", "CLIMB", "TURN", "CRUISE", "DESCEND", "HOVER", "ALIGN", "DOWN" };
  return stage < 8 ? n[stage] : "?";
}

void Navigator::startLeg(const NavIn &now) {
  const float eN = targetN - now.pN, eE = targetE - now.pE;
  dist = sqrtf(eN * eN + eE * eE);
  bestDist_ = dist;
  stuckT_ = turnT_ = arriveT_ = 0.0f;
  stage = dist > TURN_MIN_M ? STAGE_TURN : STAGE_CRUISE;
}

void Navigator::goTo(float n, float e, float h, const NavIn &now) {
  kind = NAV_GOTO;
  targetN = n; targetE = e; targetH = h;
  startLeg(now);
}

void Navigator::goHome(float homeN, float homeE, const Params &p, const NavIn &now) {
  kind = NAV_RTH;
  homeN_ = homeN; homeE_ = homeE;
  targetN = homeN; targetE = homeE;
  targetH = fcMax(now.height, p.rthAlt);       // never lower than it is flying now: that is the height it knows is clear
  startLeg(now);
  if (now.height < p.rthAlt - HEIGHT_OK_M) stage = STAGE_CLIMB;
  hoverT = 0.0f;
}

void Navigator::setMarker(float n, float e) {
  // the same place seen again: smooth it. After a gap: take the new one
  const float k = markerFresh() ? 0.3f : 1.0f;
  markerN += k * (n - markerN);
  markerE += k * (e - markerE);
  markerAge = 0.0f;
  if (kind == NAV_MARKER) { targetN = markerN; targetE = markerE; }
}

bool Navigator::goMarker(const NavIn &now) {
  if (!markerFresh()) return false;
  kind = NAV_MARKER;
  stage = STAGE_ALIGN;
  targetN = markerN; targetE = markerE;
  targetH = now.height;
  arriveT_ = 0.0f;
  pausedDown_ = false;
  return true;
}

bool Navigator::setWaypoint(int index, float n, float e, float h) {
  if (index < 0 || index >= MAX_WAYPOINTS || !fcFinite(n) || !fcFinite(e) || !fcFinite(h)) return false;
  wp_[index] = { n, e, h };
  if (index >= wpCount) wpCount = (uint8_t)(index + 1);
  return true;
}

bool Navigator::startMission(float homeN, float homeE, const NavIn &now) {
  if (!wpCount) return false;
  kind = NAV_MISSION;
  homeN_ = homeN; homeE_ = homeE;
  wpIndex = 0;
  targetN = homeN + wp_[0].n; targetE = homeE + wp_[0].e; targetH = wp_[0].h;
  startLeg(now);
  return true;
}

NavEvent Navigator::update(const NavIn &in, const Params &p, NavOut &out) {
  const float eN = targetN - in.pN, eE = targetE - in.pE;
  dist = sqrtf(eN * eN + eE * eE);
  const float speed = sqrtf(in.vN * in.vN + in.vE * in.vE);
  const float headingErr = fcWrapPi(fcAtan2(eE, eN) - in.heading);
  out.fly = false;
  out.targetN = targetN; out.targetE = targetE;
  out.speed = p.navSpeed;
  out.height = targetH;
  out.yawCmd = 0.0f;

  switch (stage) {
    case STAGE_CLIMB:
      if (fcAbs(in.height - targetH) < HEIGHT_OK_M) startLeg(in);
      break;
    case STAGE_TURN:
      turnT_ += in.dt;
      if (speed < TURN_BRAKE_SPEED) out.yawCmd = fcClamp(headingErr * YAW_GAIN, -1.0f, 1.0f);
      if (fcAbs(headingErr) < TURN_DONE_RAD || turnT_ > TURN_MAX_S) {
        stage = STAGE_CRUISE;
        bestDist_ = dist;
        stuckT_ = 0.0f;
      }
      break;
    case STAGE_CRUISE: {
      out.fly = true;
      if (dist > TURN_MIN_M) out.yawCmd = fcClamp(headingErr * YAW_GAIN, -1.0f, 1.0f);    // keep facing the way it goes
      // something stops it, or the position filter is wrong: it isn't getting there
      if (dist < bestDist_ - PROGRESS_M) { bestDist_ = dist; stuckT_ = 0.0f; }
      else if (dist > PROGRESS_NEAR_M) {
        stuckT_ += in.dt;
        if (stuckT_ > PROGRESS_S) return NAV_EV_NO_PROGRESS;
      }
      const bool here = dist < ARRIVE_M && speed < ARRIVE_SPEED && (kind == NAV_RTH || fcAbs(in.height - targetH) < HEIGHT_OK_M);
      arriveT_ = here ? arriveT_ + in.dt : 0.0f;
      if (arriveT_ < ARRIVE_S) break;
      arriveT_ = 0.0f;
      if (kind == NAV_RTH) {
        stage = STAGE_DESCEND;
        targetH = fcMin(p.rthHoverAlt, targetH);
        out.height = targetH;
      } else if (kind == NAV_GOTO) {
        return NAV_EV_ARRIVED;
      } else if (++wpIndex >= wpCount) {
        return NAV_EV_MISSION_DONE;
      } else {
        targetN = homeN_ + wp_[wpIndex].n; targetE = homeE_ + wp_[wpIndex].e; targetH = wp_[wpIndex].h;
        startLeg(in);
      }
      break;
    }
    case STAGE_DESCEND:
      out.fly = true;
      if (fcAbs(in.height - targetH) < 0.1f && fcAbs(in.vz) < 0.2f) { stage = STAGE_HOVER; hoverT = 0.0f; }
      break;
    case STAGE_HOVER:
      out.fly = true;
      hoverT += in.dt;
      break;
    case STAGE_ALIGN: {
      out.fly = true;
      out.speed = fcMin(p.navSpeed, 1.0f);
      if (markerAge > MARKER_LOST_S) return NAV_EV_LAND_NOW;
      const bool here = dist < ARRIVE_M && speed < ARRIVE_SPEED && markerFresh();
      arriveT_ = here ? arriveT_ + in.dt : 0.0f;
      if (arriveT_ >= ARRIVE_S) stage = STAGE_DOWN;
      break;
    }
    case STAGE_DOWN: {
      out.fly = true;
      out.speed = fcMin(p.navSpeed, 1.0f);
      out.descentLimit = MARKER_DESCENT;
      if (in.height < MARKER_COMMIT_M) return NAV_EV_LAND_NOW;
      if (markerAge > MARKER_LOST_S) return NAV_EV_LAND_NOW;
      // only while it can see the marker and is over it; otherwise it stays at the height it was at when that stopped
      if (markerFresh() && dist < MARKER_LINED_UP_M) {
        targetH = fcMax(in.height - 1.0f, 0.5f);
        pausedDown_ = false;
      } else if (!pausedDown_) {
        targetH = in.height;
        pausedDown_ = true;
      }
      out.height = targetH;
      break;
    }
    default:
      break;
  }
  return NAV_EV_NONE;
}
