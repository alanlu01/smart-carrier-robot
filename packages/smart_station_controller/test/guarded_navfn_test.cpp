#include <cassert>
#include <iostream>
#include "smart_station_controller/guarded_navfn.hpp"
using namespace smart_station_controller;

RouteGrid room(bool slit)
{
  RouteGrid g;g.width=g.height=120;g.resolution=.05;g.x=g.y=-3.;
  g.costs.assign(g.width*g.height,0);
  std::vector<RoutePose> obstacles;
  for (int row=0;row<g.height;++row) {
    for (int col=0;col<g.width;++col) {
      double x=g.x+(col+.5)*g.resolution,y=g.y+(row+.5)*g.resolution;
      if (row==0 || col==0 || row==g.height-1 || col==g.width-1 ||
        (slit && std::abs(y)<.051 && std::abs(x)>.175 && std::abs(x)<1.5)) {
        g.costs[row*g.width+col]=254;obstacles.push_back({x,y,0.});
      }
    }
  }
  for (int row=1;row<g.height-1;++row) {
    for (int col=1;col<g.width-1;++col) {
      if (g.costs[row*g.width+col]==254) {continue;}
      RoutePose p{g.x+(col+.5)*g.resolution,g.y+(row+.5)*g.resolution,0.};
      double d=10.;for (auto q:obstacles) {d=std::min(d,distance(p,q));}
      g.costs[row*g.width+col]=d<=.16 ? 253 :
        d<=1. ? static_cast<unsigned char>(252.*std::exp(-2.*(d-.16))) : 0;
    }
  }
  return g;
}

int main()
{
  auto g=room(true);GuardedNavfn planner;RoutePose blocked{};
  RoutePose start{0.,-2.,1.5707963267948966},goal{0.,2.,1.5707963267948966};
  auto route=planner.plan(g,start,goal,100.);
  assert(g.pathClear(route,blocked));
  double extent=0.;for (auto p:route) {extent=std::max(extent,std::abs(p.x));}
  assert(extent>1.5);  // The narrow slit is not a full-body route.
  auto opened=room(false);
  auto held=planner.plan(opened,start,goal,110.);
  assert(planner.retained && held.size()==route.size());
  assert(opened.pathClear(held,blocked));
  // A new REAL obstacle invalidates the cached route immediately.
  auto middle=held[held.size()/2];int cx,cy;assert(opened.cell(middle.x,middle.y,cx,cy));
  opened.costs[cy*opened.width+cx]=254;
  auto alternate=planner.plan(opened,start,goal,111.);
  assert(!planner.retained && opened.pathClear(alternate,blocked));
  // Fresh valid geometry does not make 25 seconds of rotating in place progress.
  auto generation=planner.generation;
  try {planner.plan(opened,start,goal,137.);} catch (const std::runtime_error &) {}
  assert(planner.stalls>0 && !planner.retained);
  assert(planner.generation>=generation);
  assert(g.costs==room(true).costs);  // Never edit the live/original snapshot.
  bool cancelled=false;
  try {GuardedNavfn p;p.plan(g,start,goal,100.,[] {return true;});}
  catch (const std::runtime_error &) {cancelled=true;}
  assert(cancelled);
  std::cout<<"guarded_navfn: narrow-route rejection, detour retention, obstacle invalidation, "
    "net-progress and cancellation passed\n";
}
