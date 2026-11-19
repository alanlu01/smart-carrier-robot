// Offline recorded-costmap probe. Never initializes ROS or publishes commands.
#include <fstream>
#include <iostream>
#include "smart_station_controller/guarded_navfn.hpp"
using namespace smart_station_controller;
int main(int argc,char ** argv)
{
  if (argc!=2) {return 2;}
  std::ifstream input(argv[1],std::ios::binary);RouteGrid g;RoutePose start{},goal{};
  input.read(reinterpret_cast<char *>(&g.width),4);input.read(reinterpret_cast<char *>(&g.height),4);
  double values[9];input.read(reinterpret_cast<char *>(values),sizeof(values));
  if (!input || g.width<3 || g.height<3 || g.width>5000 || g.height>5000) {return 3;}
  g.resolution=values[0];g.x=values[1];g.y=values[2];
  start={values[3],values[4],values[5]};goal={values[6],values[7],values[8]};
  g.costs.resize(g.width*g.height);input.read(reinterpret_cast<char *>(g.costs.data()),g.costs.size());
  if (!input) {return 4;}
  GuardedNavfn p;
  try {
    auto began=std::chrono::steady_clock::now();auto route=p.plan(g,start,goal,100.);
    double duration=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-began).count();
    RoutePose blocked{};if (!g.pathClear(route,blocked)) {return 5;}
    auto again=p.plan(g,start,goal,101.);
    std::cout<<"{\"passed\":true,\"length_m\":"<<p.length(route)<<",\"retained\":"
      <<(p.retained?"true":"false")<<",\"milliseconds\":"<<duration<<",\"points\":[";
    for (size_t i=0;i<route.size();++i) {
      if (i) {std::cout<<",";}std::cout<<"["<<route[i].x<<","<<route[i].y<<"]";
    }
    std::cout<<"]}\n";
  } catch (const std::exception & e) {std::cerr<<e.what()<<"\n";return 1;}
  return 0;
}
