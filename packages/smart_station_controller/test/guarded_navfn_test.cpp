#include <cassert>
#include <iostream>
#include "smart_station_controller/guarded_navfn.hpp"
#include "smart_station_controller/motion_stall.hpp"
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
  MotionStall tiny_motion;
  for (int i=0;i<51;++i) {
    double t=100.+i*.1;tiny_motion.command(.02,.01,.01,t);tiny_motion.odom(0.,0.,0.,t);
    assert(tiny_motion.stalled(t)==(i>=50));
  }
  assert(!tiny_motion.stalled(106.));  // stale wheels/commands cannot create evidence
  for (int i=0;i<80;++i) {
    double t=200.+i*.1;tiny_motion.command(.02,0.,.05,t);tiny_motion.odom(0.,0.,i*.005,t);
    assert(!tiny_motion.stalled(t));  // a purposeful slow turn is progress
  }
  GuardedNavfn memory;
  memory.remember({1.,1.,0.},100.);memory.remember({1.,1.,0.},110.);
  assert(memory.avoid.front().until>=200.);
  memory.cached={{0.,0.,0.},{.4,0.,0.},{.7,0.,0.},{1.,0.,0.},{1.4,0.,0.}};
  memory.motionStalled({0.,0.,0.},120.);
  assert(memory.cached.empty() && memory.avoid.size()>2);
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
  const auto remembered=planner.avoid.size();
  bool held_cancelled=false;
  try {planner.plan(opened,start,goal,110.5,[] {return true;});}
  catch (const std::runtime_error &) {held_cancelled=true;}
  assert(held_cancelled && planner.avoid.size()==remembered && !planner.cached.empty());
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
  // A mecanum can translate along a narrow lane without first rotating 90 deg.
  auto lane=room(false);
  for (int y=1;y<lane.height-1;++y) {
    for (double x:{-.325,.325}) {
      int col,row;assert(lane.cell(x,lane.y+(y+.5)*lane.resolution,col,row));
      lane.costs[row*lane.width+col]=254;
    }
  }
  RoutePose lateral_start{0.,-1.,0.},lateral_goal{0.,0.,0.};
  assert(!lane.turnClear(lateral_start,1.5707963267948966));
  GuardedNavfn lateral;
  auto sideways=lateral.plan(lane,lateral_start,lateral_goal,100.);
  assert(lane.pathClear(sideways,blocked));
  assert(std::string(lateral.heading_policy)=="goal_heading_tail");
  for (auto p:sideways) {assert(std::abs(p.yaw)<1e-8);}
  GuardedNavfn tiny;
  auto terminal=tiny.plan(lane,{0.,-.02,0.},{.005,0.,0.},100.);
  assert(std::string(tiny.heading_policy)=="short_holonomic");
  assert(lane.pathClear(terminal,blocked));
  // A zero-distance turn must still reject the real swept-body collision.
  assert(!lane.pathClear({lateral_goal,{0.,0.,1.5707963267948966}},blocked));
  // A lateral option never authorizes unknown cells or clipped body corners.
  int bx,by;assert(lane.cell(.0,-.5,bx,by));lane.costs[by*lane.width+bx]=255;
  assert(!lane.translationClear(lateral_start,lateral_goal,0.));
  std::cout<<"guarded_navfn: narrow-route rejection, detour retention, obstacle invalidation, "
    "net-progress and cancellation passed\n";
}
