#include "alt_ekf.h"

void AltEkf::reset(float altitude) {
  alt = altitude;
  vz = 0.0f;
  accBias = 0.0f;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) P[i][j] = 0.0f;
  P[0][0] = 0.25f; P[1][1] = 0.25f; P[2][2] = 0.1f;
  rejects = 0;
}

void AltEkf::predict(float accUp, float dt, float accNoise, float biasWalk) {
  if (!fcFinite(accUp)) accUp = 0.0f;
  const float a = accUp - accBias;
  alt += vz * dt + 0.5f * a * dt * dt;
  vz += a * dt;

  // P = F P F' + Q,  F = [1 dt -dt^2/2; 0 1 -dt; 0 0 1],  Q = sa^2 G G' + diag(0, 0, sb^2 dt),  G = [dt^2/2, dt, 0]'
  const float h = 0.5f * dt * dt;
  const float F[3][3] = { { 1.0f, dt, -h }, { 0.0f, 1.0f, -dt }, { 0.0f, 0.0f, 1.0f } };
  float FP[3][3], N[3][3];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) FP[i][j] = F[i][0] * P[0][j] + F[i][1] * P[1][j] + F[i][2] * P[2][j];
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) N[i][j] = FP[i][0] * F[j][0] + FP[i][1] * F[j][1] + FP[i][2] * F[j][2];
  const float sa2 = accNoise * accNoise;
  const float G[2] = { h, dt };
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++) N[i][j] += sa2 * G[i] * G[j];
  N[2][2] += biasWalk * biasWalk * dt;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) P[i][j] = 0.5f * (N[i][j] + N[j][i]);
}

bool AltEkf::updateBaro(float altMeasured, float noise) {
  if (!fcFinite(altMeasured)) return false;
  const float R = noise * noise;
  const float S = P[0][0] + R;
  const float innovation = altMeasured - alt;
  lastInnov = innovation;
  // spike rejection: more than 5 sigma AND more than 2 m away.  After 25 rejections in a row, trust the baro again.
  if (fcAbs(innovation) > 2.0f && innovation * innovation > 25.0f * S) {
    if (++rejects < 25) return false;
    reset(altMeasured);
    return true;
  }
  rejects = 0;
  const float K[3] = { P[0][0] / S, P[1][0] / S, P[2][0] / S };
  alt += K[0] * innovation;
  vz += K[1] * innovation;
  accBias += K[2] * innovation;
  accBias = fcClamp(accBias, -2.0f, 2.0f);
  const float row0[3] = { P[0][0], P[0][1], P[0][2] };
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) P[i][j] -= K[i] * row0[j];
  return true;
}
