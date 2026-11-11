#pragma once
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace smart_station_controller
{
struct SpeedEnvelope
{
  double maximum{0.15}, slow{0.06}, inner{0.20}, outer{0.40};
  void validate() const
  {
    if (!(std::isfinite(maximum) && std::isfinite(slow) && std::isfinite(inner) &&
      std::isfinite(outer) && maximum > 0 && slow > 0 && slow <= maximum &&
      inner >= 0 && outer > inner)) {
      throw std::invalid_argument("invalid station speed envelope");
    }
  }
  double limit(double distance, double external = 0., bool percentage = false) const
  {
    if (!std::isfinite(distance) || distance < 0 || !std::isfinite(external) || external < 0) {
      return 0.;
    }
    const double fraction = std::clamp((distance-inner)/(outer-inner), 0., 1.);
    const double local = slow+(maximum-slow)*fraction;
    if (external == 0.) {return local;}  // Nav2's NO_SPEED_LIMIT convention.
    return std::min(local, percentage ? maximum*std::clamp(external/100., 0., 1.) : external);
  }
};
}  // namespace smart_station_controller
