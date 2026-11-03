#include <fstream>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "nav2_a_star_rpp/theta_star.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) throw std::runtime_error("expected captured costmap text file");
  std::ifstream input(argv[1]);
  unsigned int width, height;
  double resolution, ox, oy, sx, sy, gx, gy;
  if (!(input >> width >> height >> resolution >> ox >> oy >> sx >> sy >> gx >> gy))
    throw std::runtime_error("invalid costmap header");
  nav2_costmap_2d::Costmap2D map(width, height, resolution, ox, oy, 0);
  for (unsigned int y=0; y<height; ++y) for (unsigned int x=0; x<width; ++x) {
    int value;
    if (!(input >> value)) throw std::runtime_error("truncated costmap");
    map.setCost(x, y, value);
  }
  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x=sx; start.pose.position.y=sy;
  goal.pose.position.x=gx; goal.pose.position.y=gy;
  start.pose.orientation.w=goal.pose.orientation.w=1.0;
  for (int threshold : {100, 140}) {
    theta_star::ThetaStar planner;
    planner.costmap_=&map;
    planner.lethal_cost_=threshold;
    planner.w_traversal_cost_=3.0;
    planner.allow_unknown_=false;
    planner.setStartAndGoal(start, goal);
    std::vector<coordsW> path;
    const bool found=planner.generatePath(path);
    std::cout << "Captured map, threshold=" << threshold << ": found=" << found << ", points=" << path.size() << '\n';
    if (found != (threshold==140)) throw std::runtime_error("captured obstacle regression failed");
    for (const auto &point : path) {
      unsigned int x,y;
      if (!map.worldToMap(point.x,point.y,x,y) || map.getCost(x,y)>=threshold)
        throw std::runtime_error("path enters a blocked cost cell");
    }
  }
  std::cout << "PASS: original threshold blocks the real corridor; corrected threshold finds a collision-buffered route\n";
}
