#pragma once
#include <algorithm>
#include <cmath>

namespace smart_station_controller {
// Rotation-only pulse/brake/settle. Independent from ordinary route controllers.
class AlignmentEnvelope {
public:
  void reset() { braking_ = false; bursts_ = 0; stopped_at_ = -1.; direction_ = 0.; burst_at_ = 0.; }
  double command(double error, double measured, double now, double cap = .20) {
    if (!std::isfinite(error) || !std::isfinite(measured) || !std::isfinite(now) || cap <= 0.) {
      return 0.;
    }
    const double rate = std::abs(measured), distance = std::abs(error);
    if (braking_) {
      if (rate > .03) { stopped_at_ = -1.; return 0.; }
      if (stopped_at_ < 0.) { stopped_at_ = now; }
      if (now-stopped_at_ < .75 || distance <= .06 || bursts_ >= 3) { return 0.; }
      braking_ = false;
      direction_ = 0.;
    }
    if (distance <= .06) { brake(); return 0.; }
    const double sign = error > 0. ? 1. : -1.;
    if (direction_ == 0.) { direction_ = sign; burst_at_ = now; ++bursts_; }
    // Never reverse a moving chassis on an AMCL correction. A zero command
    // plus a physical low-rate dwell precedes the next correction pulse.
    const double reserve = .06 + rate*.55 + rate*rate/(2.*.8);
    if (sign != direction_ || (now-burst_at_ >= .25 && distance <= reserve)) {
      brake(); return 0.;
    }
    return sign*std::min(cap, distance < .30 ? .14 : .20);
  }
private:
  void brake() { braking_ = true; stopped_at_ = -1.; }
  bool braking_ = false;
  unsigned bursts_ = 0;
  double stopped_at_ = -1., direction_ = 0., burst_at_ = 0.;
};
}  // namespace smart_station_controller
