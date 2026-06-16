#pragma once

class MotionController {
 public:
  void reset();
  void compute(const float raw[9], const float* baseline, float dt, float out[6]);
  bool hasMotionActivity() const;
  bool usingCalibratedDecoder() const;

 private:
  static float clampf(float v, float lo, float hi);
  static float hardZero(float v, float thr);
  static float softDeadzone(float v, float dead, float limit);
  static float lowpass(float prev, float x, float dt, float tau);
  static float axisBaseDead(int i);
  static float axisDead(int i);
  static bool calibrationAvailable();
  float filt_[6] = {};
  bool motionActive_ = false;
};
