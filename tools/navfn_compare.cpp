// Offline only: identical reconstructed costmap, installed Jazzy NavFn code.
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <vector>
#include <nav2_navfn_planner/navfn.hpp>

int main(int argc, char ** argv)
{
  if (argc!=2) {return 2;}
  std::ifstream input(argv[1],std::ios::binary);
  int dimensions[6]; input.read(reinterpret_cast<char *>(dimensions),sizeof(dimensions));
  int nx=dimensions[0],ny=dimensions[1];
  if (!input || nx<3 || ny<3 || nx>5000 || ny>5000) {return 3;}
  std::vector<unsigned char> costs(nx*ny);
  input.read(reinterpret_cast<char *>(costs.data()),costs.size());
  if (!input) {return 4;}
  std::cout<<"[";
  for (int mode=0; mode<2; ++mode) {
    std::vector<double> durations;
    std::vector<float> xs,ys;
    bool found=false;
    for (int repeat=0;repeat<7;++repeat) {
      nav2_navfn_planner::NavFn planner(nx,ny);
      planner.setCostmap(costs.data(),true,true);
      planner.setStart(dimensions+4); planner.setGoal(dimensions+2);
      auto begin=std::chrono::steady_clock::now();
      found=mode ? planner.calcNavFnAstar([] {return false;}) :
        planner.calcNavFnDijkstra([] {return false;},true);
      int length=found ? planner.calcPath(4*std::max(nx,ny)) : 0;
      auto end=std::chrono::steady_clock::now();
      if (repeat>0) {durations.push_back(std::chrono::duration<double,std::milli>(end-begin).count());}
      found=length>0;
      if (found) {
        xs.assign(planner.getPathX(),planner.getPathX()+length);
        ys.assign(planner.getPathY(),planner.getPathY()+length);
      }
    }
    std::sort(durations.begin(),durations.end());
    if (mode) {std::cout<<",";}
    std::cout<<"{\"algorithm\":\""<<(mode?"astar":"dijkstra")<<"\",\"found\":"
      <<(found?"true":"false")<<",\"median_ms\":"<<durations[durations.size()/2]<<",\"cells\":[";
    if (found) {
      for (int i=static_cast<int>(xs.size())-1;i>=0;--i) {
        if (i!=static_cast<int>(xs.size())-1) {std::cout<<",";}
        std::cout<<"["<<xs[i]<<","<<ys[i]<<"]";
      }
    }
    std::cout<<"]}";
  }
  std::cout<<"]\n";
  return 0;
}
