#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include "smart_station_controller/speed_envelope.hpp"

int main()
{
  using smart_station_controller::SpeedEnvelope;
  const auto check = [](bool valid) {if (!valid) {std::cerr << "speed envelope failed\n";std::exit(1);}};
  SpeedEnvelope envelope;envelope.validate();
  check(std::abs(envelope.limit(.1)-.06)<1e-9);
  check(std::abs(envelope.limit(.3)-.105)<1e-9);
  check(std::abs(envelope.limit(2.)-.15)<1e-9);
  double previous = .06;
  for (int i=0;i<=1000;i++) {
    const double current = envelope.limit(i*.001);
    check(current>=previous-1e-12 && current<=.15+1e-12);previous=current;
  }
  check(std::abs(envelope.limit(.3,.05)-.05)<1e-9);
  check(std::abs(envelope.limit(.3,20.,true)-.03)<1e-9);
  check(envelope.limit(std::numeric_limits<double>::quiet_NaN())==0.);
  check(envelope.limit(.3,-1.)==0.);
  envelope.outer=envelope.inner;
  bool rejected=false;try {envelope.validate();} catch (const std::invalid_argument &) {rejected=true;}
  check(rejected);
  std::cout << "station distance, continuity, external limit and invalid-input checks passed\n";
}
