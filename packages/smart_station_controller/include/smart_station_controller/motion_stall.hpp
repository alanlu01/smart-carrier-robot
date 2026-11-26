#pragma once
#include <cmath>
#include <limits>

namespace smart_station_controller
{
// Only real, fresh tiny commands and wheel observations consume this window.
class MotionStall
{
public:
  void reset() {since_=-1.;anchored_=false;}
  void command(double vx,double vy,double wz,double now)
  {
    const double speed=std::hypot(vx,vy);
    tiny_=std::isfinite(speed) && std::isfinite(wz) &&
      (speed>.005 || std::abs(wz)>.005) && speed<=.05 && std::abs(wz)<=.08;
    command_at_=now;
    if (!tiny_) {reset();}
  }
  void odom(double x,double y,double yaw,double now)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) {reset();return;}
    x_=x;y_=y;yaw_=yaw;odom_at_=now;
  }
  bool stalled(double now)
  {
    if (!tiny_ || now<command_at_ || now-command_at_>.3 ||
      now<odom_at_ || now-odom_at_>.3) {reset();return false;}
    if (!anchored_ || now<since_ || std::hypot(x_-ax_,y_-ay_)>.03 ||
      std::abs(std::atan2(std::sin(yaw_-aw_),std::cos(yaw_-aw_)))>.08)
    {ax_=x_;ay_=y_;aw_=yaw_;since_=now;anchored_=true;}
    return now-since_>=5.;
  }
private:
  double command_at_=-1e9,odom_at_=-1e9,since_=-1.;
  double x_=0.,y_=0.,yaw_=0.,ax_=0.,ay_=0.,aw_=0.;
  bool tiny_=false,anchored_=false;
};
}
