// Tilt-based sit/empty detector for a board mounted on the side of the seat cushion.
//
// Why tilt: a person sitting still makes no more vibration than an empty chair, so
// "is it moving" can't tell them apart. But sitting squishes the cushion and tilts
// the board by ~0.7 m/s^2, while moving an empty chair only shifts it ~0.1.
//
// How it works, one sample at a time at 50 Hz:
//   1. Motion = 1 s std of accel magnitude > 0.035 m/s^2, or of gyro magnitude > 0.4 deg/s.
//   2. After motion, wait 3 s of quiet, then average (ax, ay, az) over the last 2 s.
//      That's the resting tilt v. It's checked once per motion event.
//   3. d = distance from v to the empty-chair baseline.
//      EMPTY:   d > 0.2 -> SITTING (remember v as the sitting tilt).
//               otherwise stay EMPTY and move the baseline to v, which absorbs bumps,
//               rolls, and the foam not fully springing back.
//      SITTING: d < 0.2 and v is closer to the baseline than to the sitting tilt -> EMPTY.
//
// Plain C++ with no Arduino calls, so the same code runs on the ESP32 and in
// tools/replay_csv.cpp on a PC.
#pragma once

#include <stdint.h>

class OccupancyDetector {
 public:
  enum State : uint8_t { EMPTY = 0, SITTING = 1 };

  struct Config {
    float tiltThreshold = 0.2f;     // m/s^2, distance that counts as "different tilt"
    float accelMotion = 0.035f;     // m/s^2, 1 s std of accel magnitude
    float gyroMotion = 0.4f;        // deg/s, 1 s std of gyro magnitude
    uint16_t quietSamples = 150;    // 3 s at 50 Hz
  };

  OccupancyDetector() : OccupancyDetector(Config()) {}
  explicit OccupancyDetector(const Config &cfg);

  // Feed one sample (m/s^2 and deg/s). Call at 50 Hz.
  // Returns true on the sample where the state changes.
  bool update(float ax, float ay, float az, float gx, float gy, float gz);

  State state() const { return state_; }
  bool occupied() const { return state_ == SITTING; }
  float lastDistance() const { return lastD_; }  // d from the most recent tilt check

 private:
  static const int MOTION_WIN = 50;  // 1 s
  static const int TILT_WIN = 100;   // 2 s

  Config cfg_;
  State state_ = EMPTY;

  float amag_[MOTION_WIN], gmag_[MOTION_WIN];
  float tilt_[TILT_WIN][3];
  int motionHead_ = 0, tiltHead_ = 0;
  uint32_t samples_ = 0;

  uint32_t quiet_ = 0;
  bool moved_ = true;  // check tilt once right after startup to set the baseline
  bool haveBase_ = false;
  float base_[3] = {0, 0, 0};
  float sit_[3] = {0, 0, 0};
  float lastD_ = 0;

  static float stddev(const float *x, int n);
  static float dist(const float *a, const float *b);
};
