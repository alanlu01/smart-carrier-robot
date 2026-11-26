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
      // Each destination encodes the heading of that translation. A duplicate
      // endpoint is an explicit stopped rotation, including the final yaw.
      const double yaw=b.yaw;
      blocked={a.x,a.y,previous};
      if (!turnClear(blocked,yaw)) {return false;}
      blocked={(a.x+b.x)/2.,(a.y+b.y)/2.,yaw};
      if (length>=1e-7 && !translationClear(a,b,yaw,cancel)) {return false;}
      previous=yaw;
    }
    blocked={path.back().x,path.back().y,previous};
    return turnClear(blocked,path.back().yaw);
  }
  bool translationClear(RoutePose a,RoutePose b,double heading,
    const std::function<bool()> & cancel=[] {return false;}) const
  {
    if (!boxClear({a.x,a.y,heading}) || !boxClear({b.x,b.y,heading})) {return false;}
    const double dx=b.x-a.x,dy=b.y-a.y,c=std::cos(heading),s=std::sin(heading);
    const RoutePose mid{(a.x+b.x)/2.,(a.y+b.y)/2.,heading};
    const double ex=.25*std::abs(c)+.19*std::abs(s)+std::abs(dx)/2.;
    const double ey=.25*std::abs(s)+.19*std::abs(c)+std::abs(dy)/2.;
    int lx,ly,hx,hy;
    if (!cell(mid.x-ex-1e-9,mid.y-ey-1e-9,lx,ly) ||
      !cell(mid.x+ex+1e-9,mid.y+ey+1e-9,hx,hy)) {return false;}
    // SAT of the exact rectangle+translation convex sweep against cell squares.
    // Include its oblique edge; a diagonal bounding rectangle is too restrictive.
    const double axes[5][2]={{1.,0.},{0.,1.},{c,s},{-s,c},{-dy,dx}};
    for (int row=ly;row<=hy;++row) {
      if (cancel()) {return false;}
      for (int col=lx;col<=hx;++col) {
        if (costs[row*width+col]<254) {continue;}
        const double px=x+(col+.5)*resolution-mid.x,py=y+(row+.5)*resolution-mid.y;
        bool overlaps=true;
        for (const auto & axis:axes) {
          const double nx=axis[0],ny=axis[1];
          const double radius=.25*std::abs(nx*c+ny*s)+.19*std::abs(-nx*s+ny*c)+
            std::abs(nx*dx+ny*dy)/2.+resolution/2.*(std::abs(nx)+std::abs(ny));
          if (std::abs(nx*px+ny*py)>radius+1e-12) {overlaps=false;break;}
        }
        if (overlaps) {return false;}
      }
    }
    return true;
  }
};

class GuardedNavfn
{
public:
  struct Avoid {RoutePose point;double until;unsigned hits=1;};
  std::vector<RoutePose> cached;
  std::vector<Avoid> avoid;
  RoutePose goal{};
  bool has_goal=false,retained=false;
  double best_remaining=0.,last_progress=0.,last_now=0.;
  unsigned generation=0,repairs=0,stalls=0;
  const char * heading_policy="tangent";

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
      if (distance(item.point,p)<.10) {
        ++item.hits;item.until=now+std::min(180.,45.*item.hits);return;
      }
    }
    if (avoid.size()>=32) {avoid.erase(avoid.begin());}
    avoid.push_back({p,now+45.});
  }
  void rememberCorridor(const std::vector<RoutePose> & path,RoutePose p,double now)
  {
    remember(p,now);double selected=-1.;
    for (size_t i=0;i<path.size();++i) {
      if (distance(path[i],p)>.55) {continue;}
      if (selected>=0 && distance(path[i],path[static_cast<size_t>(selected)])<.20) {continue;}
      remember(path[i],now);selected=static_cast<double>(i);
    }
  }
  void motionStalled(RoutePose start,double now)
  {
    if (cached.empty()) {return;}
    RoutePose ahead=cached.back();double walked=0.;
    for (size_t i=1;i<cached.size();++i) {
      walked+=distance(cached[i-1],cached[i]);if (walked>=.6) {ahead=cached[i];break;}
    }
    if (distance(ahead,start)>.30) {rememberCorridor(cached,ahead,now);}
    cached.clear();++stalls;
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
      cached.clear();goal=target;has_goal=true;last_progress=now;
    }
    avoid.erase(std::remove_if(avoid.begin(),avoid.end(),[now](const auto & a) {
      return now>=a.until;}),avoid.end());
    RoutePose blocked{};
    const auto began=std::chrono::steady_clock::now();
    auto interrupted=[&] {return cancel() || std::chrono::steady_clock::now()-began>
      std::chrono::milliseconds(250);};
    if (!cached.empty()) {
      auto nearest=std::min_element(cached.begin(),cached.end(),[&](auto a,auto b) {
        return distance(start,a)<distance(start,b);});
      if (distance(start,*nearest)<=.50) {
        std::vector<RoutePose> tail{start};
        if (nearest+1!=cached.end()) {tail.insert(tail.end(),nearest+1,cached.end());}
        else {tail.push_back(target);}
        const bool cache_clear=grid.pathClear(tail,blocked,interrupted);
        if (interrupted()) {throw std::runtime_error("Guarded route validation interrupted");}
        if (cache_clear) {
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
          rememberCorridor(tail,blocked,now);++stalls;
        } else if (distance(blocked,start)>.30 && distance(blocked,target)>.30) {
          rememberCorridor(tail,blocked,now);
        }
      }
      cached.clear();
    }
    // Avoid grid-centre zigzags for centimetre terminal moves. The exact same
    // swept footprint and stopped terminal turn are still mandatory.
    if (distance(start,target)<=.10 && !interrupted()) {
      for (double heading:{start.yaw,target.yaw}) {
        std::vector<RoutePose> direct{start,{target.x,target.y,heading},target};
        if (grid.pathClear(direct,blocked,interrupted) && !interrupted()) {
          cached=direct;best_remaining=length(cached);last_progress=now;++generation;
          heading_policy="short_holonomic";return cached;
        }
      }
    }
    for (unsigned attempt=0;attempt<8 && !interrupted();++attempt) {
      auto costs=grid.costs;
      for (const auto & item:avoid) {
        int cx,cy;if (!grid.cell(item.point.x,item.point.y,cx,cy)) {continue;}
        const int radius=static_cast<int>(std::ceil(.20/grid.resolution));
        for (int row=std::max(0,cy-radius);row<=std::min(grid.height-1,cy+radius);++row) {
          for (int col=std::max(0,cx-radius);col<=std::min(grid.width-1,cx+radius);++col) {
            const RoutePose p{grid.x+(col+.5)*grid.resolution,grid.y+(row+.5)*grid.resolution,0.};
            if (distance(p,item.point)<=.20 && distance(p,start)>.30 && distance(p,target)>.30) {
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
        if (distance(start,p)>.075 && distance(target,p)>.075 &&
          distance(candidate.back(),p)>1e-5) {candidate.push_back(p);}
      }
      candidate.push_back(target);
      for (size_t i=1;i<candidate.size();++i) {
        candidate[i].yaw=std::atan2(candidate[i].y-candidate[i-1].y,
          candidate[i].x-candidate[i-1].x);
      }
      candidate.push_back(target);
      auto accepted=[&](const std::vector<RoutePose> & route,const char * policy) {
        if (!grid.pathClear(route,blocked,interrupted) || interrupted()) {return false;}
        cached=route;best_remaining=length(cached);last_progress=now;++generation;
        heading_policy=policy;return true;
      };
      if (accepted(candidate,"tangent")) {return cached;}
      const auto tangent_blocked=blocked;
      // Only the final 1.5 m may hold a heading. Ordinary long-route driving
      // retains its tangent policy; this does not change MPPI lateral limits.
      for (bool hold_target:{true,false}) {
        auto alternate=candidate;double remaining=0.;size_t first=alternate.size()-1;
        for (size_t i=alternate.size()-1;i>0;--i) {
          remaining+=distance(alternate[i-1],alternate[i]);
          if (remaining>1.5) {break;}first=i;
        }
        if (first>=alternate.size()-1) {continue;}
        const double heading=hold_target ? target.yaw : alternate[first-1].yaw;
        for (size_t i=first;i+1<alternate.size();++i) {alternate[i].yaw=heading;}
        if (accepted(alternate,hold_target ? "goal_heading_tail" : "arrival_heading_tail")) {
          return cached;
        }
      }
      blocked=tangent_blocked;
      if (interrupted() || distance(blocked,start)<.30 || distance(blocked,target)<.30) {break;}
      remember(blocked,now);++repairs;
    }
    throw std::runtime_error("No footprint-safe Dijkstra route in bounded planning budget");
  }
};
}  // namespace smart_station_controller
