#include "controllers/MotionController.h"

#include <Arduino.h>
#include <math.h>

#include "Config.h"

#if defined(__has_include)
#if __has_include("CalibrationData.h")
#define CAD_MOUSE_HAS_CALIBRATION_DATA 1
#include "CalibrationData.h"
#endif
#endif

#ifndef CAD_MOUSE_HAS_CALIBRATION_DATA
namespace CalibrationData {
constexpr bool AVAILABLE = false;
constexpr float MATRIX[6][9] = {};
constexpr float DEADZONE[6] = {};
}  // namespace CalibrationData
#endif

namespace {
enum RawIndex {
  RAW_MAG1_X = 0,
  RAW_MAG1_Y,
  RAW_MAG1_Z,
  RAW_MAG2_X,
  RAW_MAG2_Y,
  RAW_MAG2_Z,
  RAW_MAG3_X,
  RAW_MAG3_Y,
  RAW_MAG3_Z
};

enum AxisIndex {
  AXIS_TX = 0,
  AXIS_TY,
  AXIS_TZ,
  AXIS_RX,
  AXIS_RY,
  AXIS_RZ
};

// Sub-count residual below which an axis is snapped to zero so it truly settles
// after the filter's release tail. The HID layer casts to int16, so anything
// under 1.0 is already reported as zero; snapping here lets motionActive_ clear
// (and idle/sleep proceed) instead of latching on a vanishing filter value.
constexpr float kAxisSettle = 1.0;
}  // namespace

void MotionController::reset() {
  for (int i = 0; i < 6; i++) {
    filt_[i] = 0.0;
  }
  motionActive_ = false;
}

float MotionController::clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float MotionController::hardZero(float v, float thr) {
  return (fabs(v) < thr) ? 0.0 : v;
}

float MotionController::softDeadzone(float v, float dead, float limit) {
  // Continuous dead zone: inputs within +/-dead map to zero, inputs beyond it
  // ramp up *from* zero (rather than jumping to ~dead like a hard gate). The
  // surviving range [dead, limit] is rescaled back to [0, limit] so the axis
  // still reaches full scale. This removes the onset "pop" and, paired with the
  // low-pass, damps threshold flicker on noisy, large-dead-zone axes.
  const float a = fabs(v);
  if (a <= dead) return 0.0;
  const float span = limit - dead;
  if (span <= 0.0) return v;  // degenerate dead zone >= limit: pass through
  const float scaled = (a - dead) * (limit / span);
  return (v < 0.0) ? -scaled : scaled;
}

float MotionController::lowpass(float prev, float x, float dt, float tau) {
  if (tau <= 0.0) return x;
  const float a = dt / (tau + dt);
  return prev + a * (x - prev);
}

float MotionController::axisBaseDead(int i) {
  return (i < 3) ? Config::DEAD_T : Config::DEAD_R;
}

bool MotionController::calibrationAvailable() {
  return CalibrationData::AVAILABLE;
}

bool MotionController::usingCalibratedDecoder() const {
  return calibrationAvailable();
}

float MotionController::axisDead(int i) {
  if (calibrationAvailable()) {
    return CalibrationData::DEADZONE[i];
  }
  return axisBaseDead(i);
}

void MotionController::compute(const float raw[9], const float* baseline, float dt,
                               float out[6]) {
  // Baseline subtraction converts magnetic deltas around the calibrated rest pose.
  float delta[9] = {};
  for (int i = 0; i < 9; i++) {
    delta[i] = raw[i] - baseline[i];
  }

  float y[6] = {};

  if (calibrationAvailable()) {
    for (int axis = 0; axis < 6; axis++) {
      float v = 0.0;
      for (int rawAxis = 0; rawAxis < 9; rawAxis++) {
        v += CalibrationData::MATRIX[axis][rawAxis] * delta[rawAxis];
      }
      y[axis] = v;
    }
  } else {
    const float mag1x = delta[RAW_MAG1_X];
    const float mag1y = delta[RAW_MAG1_Y];
    const float mag1z = delta[RAW_MAG1_Z];
    const float mag2x = delta[RAW_MAG2_X];
    const float mag2y = delta[RAW_MAG2_Y];
    const float mag2z = delta[RAW_MAG2_Z];
    const float mag3x = delta[RAW_MAG3_X];
    const float mag3y = delta[RAW_MAG3_Y];
    const float mag3z = delta[RAW_MAG3_Z];

    // Translation:
    //   Tx = (mag1x + mag2x + mag3x) / 3
    //   Ty = (mag1y + mag2y + mag3y) / 3
    //   Tz = (mag1z + mag2z + mag3z) / 3
    const float tx = (mag1x + mag2x + mag3x) / 3.0;
    const float ty = (mag1y + mag2y + mag3y) / 3.0;
    const float tz = (mag1z + mag2z + mag3z) / 3.0;

    // Physical PCB layout:
    // MAG2 = top left, MAG3 = top right, MAG1 = bottom.
    const float mag2PosX = -0.5;
    const float mag2PosY = sqrt(3.0) / 6.0;

    const float mag3PosX = 0.5;
    const float mag3PosY = sqrt(3.0) / 6.0;

    const float mag1PosX = 0.0;
    const float mag1PosY = -sqrt(3.0) / 3.0;

    // Rotation estimates:
    //   Ry = mag3z - mag2z
    //     right sensor minus left sensor
    //     -> side to side tilt across the top edge
    //
    //   Rx = sqrt(3) * (mag2z + mag3z - 2 * mag1z) / 3
    //     top pair minus bottom sensor
    //     -> front/back tilt of the triangle
    const float rx = (sqrt(3.0) * (mag2z + mag3z - 2.0 * mag1z)) / 3.0;
    const float ry = (mag3z - mag2z);

    //   Rz = sum_i (posXi * magYi - posYi * magXi)
    // Each sensor contributes according to its x/y position in the triangle.
    const float swirlNum =
        (mag2PosX * mag2y - mag2PosY * mag2x) +
        (mag3PosX * mag3y - mag3PosY * mag3x) +
        (mag1PosX * mag1y - mag1PosY * mag1x);
    const float rz = swirlNum;

    // Apply sign fixes and gains.
    y[AXIS_TX] = Config::SIGN_AXIS[AXIS_TX] * tx * Config::GAIN_T[AXIS_TX];
    y[AXIS_TY] = Config::SIGN_AXIS[AXIS_TY] * ty * Config::GAIN_T[AXIS_TY];
    y[AXIS_TZ] = Config::SIGN_AXIS[AXIS_TZ] * tz * Config::GAIN_T[AXIS_TZ];
    y[AXIS_RX] = Config::SIGN_AXIS[AXIS_RX] * rx * Config::GAIN_R[AXIS_RX - 3];
    y[AXIS_RY] = Config::SIGN_AXIS[AXIS_RY] * ry * Config::GAIN_R[AXIS_RY - 3];
    y[AXIS_RZ] = Config::SIGN_AXIS[AXIS_RZ] * rz * Config::GAIN_R[AXIS_RZ - 3];
  }

  // Soft dead zone -> filter -> clamp -> settle.
  // Each axis is shaped through its dead zone *before* filtering: rest noise
  // collapses to zero (so the filter holds at rest) and real motion ramps in
  // continuously. The low-pass then smooths the shaped command, and a sub-count
  // settle snap lets the release tail reach a true zero.
  motionActive_ = false;
  for (int i = 0; i < 6; i++) {
    const float dead = axisDead(i);
    const float shaped = softDeadzone(y[i], dead, Config::AXIS_LIMIT);

    filt_[i] = lowpass(filt_[i], shaped, dt, Config::SMOOTH_TAU_S[i]);

    const float limited =
        clampf(filt_[i], -Config::AXIS_LIMIT, Config::AXIS_LIMIT);
    out[i] = hardZero(limited, kAxisSettle);
    if (out[i] != 0.0) {
      motionActive_ = true;
    }
  }
}

bool MotionController::hasMotionActivity() const { return motionActive_; }
