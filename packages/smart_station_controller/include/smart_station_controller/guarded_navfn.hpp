// NavFn Dijkstra on a private snapshot, with continuous padded-body sweeps.
#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <vector>
#include <nav2_navfn_planner/navfn.hpp>

namespace smart_station_controller
{
struct RoutePose {double x, y, yaw;};
inline double wrap(double x) {return std::atan2(std::sin(x),std::cos(x));}
inline double distance(RoutePose a, RoutePose b) {return std::hypot(a.x-b.x,a.y-b.y);}
struct RouteGrid
{
  int width{}, height{};
  double resolution{}, x{}, y{};
  std::vector<unsigned char> costs;
  bool cell(double wx,double wy,int & col,int & row) const
  {
    if (!std::isfinite(wx) || !std::isfinite(wy) || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(resolution) || resolution<=0.) {return false;}
    col=static_cast<int>(std::floor((wx-x)/resolution));
    row=static_cast<int>(std::floor((wy-y)/resolution));
    return col>=0 && row>=0 && col<width && row<height;
  }
  bool boxClear(RoutePose p,double hx=.25,double hy=.19) const
  {
    if (width<3 || height<3 || !std::isfinite(resolution) || resolution<=0. ||
      costs.size()!=static_cast<size_t>(width*height) || !std::isfinite(p.x) ||
      !std::isfinite(p.y) || !std::isfinite(p.yaw)) {return false;}
    const double c=std::cos(p.yaw),s=std::sin(p.yaw),ac=std::abs(c),as=std::abs(s);
    const double ex=hx*ac+hy*as,ey=hx*as+hy*ac,h=resolution/2.;
    int lx,ly,hix,hiy;
    if (!cell(p.x-ex-1e-9,p.y-ey-1e-9,lx,ly) ||
      !cell(p.x+ex+1e-9,p.y+ey+1e-9,hix,hiy)) {return false;}
    for (int row=ly;row<=hiy;++row) {
      for (int col=lx;col<=hix;++col) {
        // 253 is inscribed inflation, not a physical object. Unknown is blocked.
        if (costs[row*width+col]<254) {continue;}
        const double dx=x+(col+.5)*resolution-p.x,dy=y+(row+.5)*resolution-p.y;
        if (std::abs(dx)<=ex+h+1e-12 && std::abs(dy)<=ey+h+1e-12 &&
          std::abs(c*dx+s*dy)<=hx+h*(ac+as)+1e-12 &&
          std::abs(-s*dx+c*dy)<=hy+h*(ac+as)+1e-12) {return false;}
      }
    }
    return true;
  }
  bool turnClear(RoutePose p,double yaw) const
  {
    const double delta=wrap(yaw-p.yaw);
    const int steps=std::max(1,static_cast<int>(std::ceil(std::abs(delta)/.05)));
    // Enclose all corner arcs between orientation samples.
    const double pad=std::hypot(.25,.19)*(1-std::cos(std::abs(delta/steps)/2.))+1e-9;
    for (int i=0;i<=steps;++i) {
      RoutePose q{p.x,p.y,p.yaw+delta*i/steps};
      if (!boxClear(q,.25+pad,.19+pad)) {return false;}
    }
    return true;
  }
  bool pathClear(const std::vector<RoutePose> & path,RoutePose & blocked,
    const std::function<bool()> & cancel=[] {return false;}) const
  {
    if (path.empty()) {return false;}
    double previous=path.front().yaw;
    for (size_t i=1;i<path.size();++i) {
      if (cancel()) {return false;}
      const auto a=path[i-1],b=path[i];
      const double length=distance(a,b);
      if (length<1e-7) {continue;}
      const double yaw=std::atan2(b.y-a.y,b.x-a.x);
      blocked={a.x,a.y,previous};
      if (!turnClear(blocked,yaw)) {return false;}
      // Segment-aligned rectangle is the EXACT convex body translation sweep.
      blocked={(a.x+b.x)/2.,(a.y+b.y)/2.,yaw};
      if (!boxClear(blocked,.25+length/2.,.19)) {return false;}
      previous=yaw;
    }
    blocked={path.back().x,path.back().y,previous};
    return turnClear(blocked,path.back().yaw);
  }
};

class GuardedNavfn
{
public:
  struct Avoid {RoutePose point;double until;};
  std::vector<RoutePose> cached;
  std::vector<Avoid> avoid;
  RoutePose goal{};
  bool has_goal=false,retained=false;
  double best_remaining=0.,last_progress=0.,last_now=0.;
  unsigned generation=0,repairs=0,stalls=0;

  void reset() {cached.clear();avoid.clear();has_goal=false;}
  static double length(const std::vector<RoutePose> & path)
  {
    double result=0.;
    for (size_t i=1;i<path.size();++i) {result+=distance(path[i-1],path[i]);}
    return result;
  }
  void remember(RoutePose p,double now)
  {
    for (auto & item:avoid) {
      if (distance(item.point,p)<.10) {item.until=now+45.;return;}
    }
    if (avoid.size()>=32) {avoid.erase(avoid.begin());}
    avoid.push_back({p,now+45.});
  }
  std::vector<RoutePose> plan(const RouteGrid & grid,RoutePose start,RoutePose target,
    double now,const std::function<bool()> & cancel=[] {return false;})
  {
    retained=false;repairs=0;
    if (grid.width<3 || grid.height<3 || !std::isfinite(grid.resolution) || grid.resolution<=0. ||
      grid.costs.size()!=static_cast<size_t>(grid.width*grid.height)) {
      throw std::runtime_error("Invalid route costmap snapshot");
    }
    if (!std::isfinite(now)) {throw std::runtime_error("Invalid route clock");}
    if (now<last_now) {reset();}
    last_now=now;
    if (!has_goal || distance(goal,target)>.01 || std::abs(wrap(goal.yaw-target.yaw))>.01) {
      reset();goal=target;has_goal=true;last_progress=now;
    }
    avoid.erase(std::remove_if(avoid.begin(),avoid.end(),[now](const auto & a) {
      return now>=a.until;}),avoid.end());
    RoutePose blocked{};
    if (!cached.empty()) {
      auto nearest=std::min_element(cached.begin(),cached.end(),[&](auto a,auto b) {
        return distance(start,a)<distance(start,b);});
      if (distance(start,*nearest)<=.50) {
        std::vector<RoutePose> tail{start};
        if (nearest+1!=cached.end()) {tail.insert(tail.end(),nearest+1,cached.end());}
        else {tail.push_back(target);}
        if (grid.pathClear(tail,blocked,cancel)) {
          const double remaining=length(tail);
          if (remaining<=best_remaining-.15) {best_remaining=remaining;last_progress=now;}
          if (now-last_progress<25.) {cached=tail;retained=true;return cached;}
          // Turning in place never refreshes arc-length progress. Avoid the
          // next blocked corridor privately and request another bounded route.
          double ahead=0.;blocked=tail.back();
          for (size_t i=1;i<tail.size();++i) {
            ahead+=distance(tail[i-1],tail[i]);
            if (ahead>=.6) {blocked=tail[i];break;}
          }
          remember(blocked,now);++stalls;
        } else if (distance(blocked,start)>.30 && distance(blocked,target)>.30) {
          remember(blocked,now);
        }
      }
      cached.clear();
    }
    const auto began=std::chrono::steady_clock::now();
    auto interrupted=[&] {return cancel() || std::chrono::steady_clock::now()-began>
      std::chrono::milliseconds(250);};
    for (unsigned attempt=0;attempt<8 && !interrupted();++attempt) {
      auto costs=grid.costs;
      for (const auto & item:avoid) {
        int cx,cy;if (!grid.cell(item.point.x,item.point.y,cx,cy)) {continue;}
        const int radius=static_cast<int>(std::ceil(.15/grid.resolution));
        for (int row=std::max(0,cy-radius);row<=std::min(grid.height-1,cy+radius);++row) {
          for (int col=std::max(0,cx-radius);col<=std::min(grid.width-1,cx+radius);++col) {
            const RoutePose p{grid.x+(col+.5)*grid.resolution,grid.y+(row+.5)*grid.resolution,0.};
            if (distance(p,item.point)<=.15 && distance(p,start)>.30 && distance(p,target)>.30) {
              costs[row*grid.width+col]=254;
            }
          }
        }
      }
      int begin[2],end[2];
      if (!grid.cell(start.x,start.y,begin[0],begin[1]) ||
        !grid.cell(target.x,target.y,end[0],end[1])) {break;}
      nav2_navfn_planner::NavFn native(grid.width,grid.height);
      native.setCostmap(costs.data(),true,false);
      native.setStart(end);native.setGoal(begin);
      if (!native.calcNavFnDijkstra(interrupted,true) || interrupted()) {break;}
      const int count=native.calcPath(4*std::max(grid.width,grid.height));
      if (count<1) {break;}
      std::vector<RoutePose> candidate{start};
      for (int i=count-1;i>=0;--i) {
        RoutePose p{grid.x+(native.getPathX()[i]+.5)*grid.resolution,
          grid.y+(native.getPathY()[i]+.5)*grid.resolution,0.};
        if (distance(candidate.back(),p)>1e-5) {candidate.push_back(p);}
      }
      candidate.push_back(target);
      if (grid.pathClear(candidate,blocked,interrupted) && !interrupted()) {
        cached=candidate;best_remaining=length(cached);last_progress=now;++generation;return cached;
      }
      if (interrupted() || distance(blocked,start)<.30 || distance(blocked,target)<.30) {break;}
      remember(blocked,now);++repairs;
    }
    throw std::runtime_error("No footprint-safe Dijkstra route in bounded planning budget");
  }
};
}  // namespace smart_station_controller
