#include <cassert>
#include <cmath>
#include <deque>
#include "smart_station_controller/alignment_envelope.hpp"
using smart_station_controller::AlignmentEnvelope;
int main() {
  for (double initial : {-.20, .20, -1.92, 1.92}) {
    for (double gain : {1.,1.5}) {
      AlignmentEnvelope envelope;
      double angle=initial,rate=0.,measured=0.;
      std::deque<double> delay(20,0.);
      for (int i=0;i<2200;++i) {
        if (i%50==0) { measured=rate; }
        const double cmd=envelope.command(-angle,measured,i*.01);
        assert(std::abs(cmd)<=.20);
        delay.push_back(std::abs(cmd)<.11 ? 0. : gain*cmd);
        const double target=delay.front(); delay.pop_front();
        rate+=std::clamp(target-rate,-.8*.01,.8*.01);
        angle+=rate*.01;
      }
      assert(std::abs(angle)<=.06);
      assert(std::abs(rate)<.03);
    }
  }
  AlignmentEnvelope envelope;
  assert(envelope.command(1.,0.,0.)>0.);
  assert(envelope.command(-.2,.25,.5)==0.);
  assert(envelope.command(-.2,0.,.6)==0.);
  assert(envelope.command(-.2,0.,1.4)<0.);
  assert(envelope.command(NAN,0.,1.5)==0.);
}
