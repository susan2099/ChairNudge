#include "occupancy.h"
#include "../OccupancyTimer.h"

#include <math.h>

OccupancyDetector::OccupancyDetector(const Config &cfg) : cfg_(cfg) {}

// Sample standard deviation (n - 1), same as pandas rolling().std().
float OccupancyDetector::stddev(const float *x, int n) {
  float mean = 0;
  for (int i = 0; i < n; i++) mean += x[i];
  mean /= n;
  float ss = 0;
  for (int i = 0; i < n; i++) ss += (x[i] - mean) * (x[i] - mean);
  return sqrtf(ss / (n - 1));
}

float OccupancyDetector::dist(const float *a, const float *b) {
  float dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
  return sqrtf(dx * dx + dy * dy + dz * dz);
}

bool OccupancyDetector::update(float ax, float ay, float az, float gx, float gy, float gz) {
  State before = state_;

  // 1. Motion check over the last 1 s (no motion until the window is full).
  amag_[motionHead_] = sqrtf(ax * ax + ay * ay + az * az);
  gmag_[motionHead_] = sqrtf(gx * gx + gy * gy + gz * gz);
  motionHead_ = (motionHead_ + 1) % MOTION_WIN;
  samples_++;

  bool motion = false;
  if (samples_ >= MOTION_WIN) {
    motion = stddev(amag_, MOTION_WIN) > cfg_.accelMotion ||
             stddev(gmag_, MOTION_WIN) > cfg_.gyroMotion;
  }
  if (motion) {
    quiet_ = 0;
    moved_ = true;
  } else {
    quiet_++;
  }

  // 2. Once things have been quiet for 3 s after a motion, check the resting tilt.
  //    The average uses the 2 s before this sample, all of which were quiet.
  if (moved_ && quiet_ >= cfg_.quietSamples) {
    float v[3] = {0, 0, 0};
    for (int i = 0; i < TILT_WIN; i++)
      for (int k = 0; k < 3; k++) v[k] += tilt_[i][k];
    for (int k = 0; k < 3; k++) v[k] /= TILT_WIN;

    if (!haveBase_) {
      for (int k = 0; k < 3; k++) base_[k] = v[k];
      haveBase_ = true;
    }
    float d = dist(v, base_);
    lastD_ = d;

    // 3. Decide.
    if (state_ == EMPTY) {
      if (d > cfg_.tiltThreshold) {
        state_ = SITTING;
        for (int k = 0; k < 3; k++) sit_[k] = v[k];
        OccupancyTimer::UserSat();
      } else {
        for (int k = 0; k < 3; k++) base_[k] = v[k];  // chair moved or foam settled
      }
    } else if (d < cfg_.tiltThreshold && d < dist(v, sit_)) {
      state_ = EMPTY;
      for (int k = 0; k < 3; k++) base_[k] = v[k];
      OccupancyTimer::UserStood();
    }
    moved_ = false;
  }

  // Add this sample to the tilt window after the check, so the check only sees past samples.
  tilt_[tiltHead_][0] = ax;
  tilt_[tiltHead_][1] = ay;
  tilt_[tiltHead_][2] = az;
  tiltHead_ = (tiltHead_ + 1) % TILT_WIN;

  return state_ != before;
}
