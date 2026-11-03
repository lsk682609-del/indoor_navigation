#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "nav2_a_star_rpp/theta_star.hpp"

void require(bool result, const char * message)
{
  if (!result) throw std::runtime_error(message);
}

geometry_msgs::msg::PoseStamped pose(double x, double y)
{
  geometry_msgs::msg::PoseStamped value;
  value.pose.position.x = x;
  value.pose.position.y = y;
  value.pose.orientation.w = 1.0;
  return value;
}

class EdgePlanner : public theta_star::ThetaStar
{
public:
  using theta_star::ThetaStar::losCheck;
};

int main()
{
  nav2_costmap_2d::Costmap2D map(20, 20, 1.0, 0.0, 0.0, 254);
  EdgePlanner planner;
  planner.costmap_ = &map;
  planner.lethal_cost_ = 100;
  planner.allow_unknown_ = false;
  planner.start_relax_radius_ = 0.0;
  std::vector<coordsW> path;
  // No expansion is possible; the search must stop without following stale node pointers.
  map.setCost(2, 2, 0);
  map.setCost(17, 17, 0);
  planner.setStartAndGoal(pose(2.5, 2.5), pose(17.5, 17.5));
  require(!planner.generatePath(path) && path.empty(), "isolated start must return no path");
  for (int cycle = 0; cycle < 100; ++cycle) {
    // Reuse the planner across failures, rolling-window changes and recoveries.
    map.resetMap(0, 0, 20, 20);
    for (unsigned int y = 0; y < 20; ++y)
      for (unsigned int x = 0; x < 20; ++x) map.setCost(x, y, x == 10 ? 254 : 0);
    planner.setStartAndGoal(pose(2.5, 2.5), pose(17.5, 17.5));
    require(!planner.generatePath(path) && path.empty(), "closed wall must be unreachable");
    map.setCost(10, 10, 0);
    planner.setStartAndGoal(pose(2.5, 2.5), pose(17.5, 17.5));
    require(planner.generatePath(path) && !path.empty(), "open doorway must allow recovery");
    require(std::hypot(path.front().x - 2.5, path.front().y - 2.5) < 1e-6, "wrong start");
    require(std::hypot(path.back().x - 17.5, path.back().y - 17.5) < 1e-6, "wrong goal");
  }
  planner.setStartAndGoal(pose(-1.0, 2.5), pose(17.5, 17.5));
  require(!planner.generatePath(path) && path.empty(), "out-of-map start must fail safely");
  planner.setStartAndGoal(pose(2.5, 2.5), pose(25.0, 17.5));
  require(!planner.generatePath(path) && path.empty(), "out-of-map goal must fail safely");
  planner.setStartAndGoal(pose(2.5, 2.5), pose(2.5, 2.5));
  require(planner.generatePath(path) && !path.empty(), "same-cell path must succeed");
  // Bresenham LOS probes x-1 / y-1 along the first row/column when the edge is blocked.
  for (int i = 0; i < 20; ++i) {map.setCost(0, i, 254); map.setCost(i, 0, 254);}
  double cost = 0.0;
  require(!planner.losCheck(0, 1, 0, 18, cost), "blocked left border must be unsafe");
  require(!planner.losCheck(1, 0, 18, 0, cost), "blocked bottom border must be unsafe");
  map.resizeMap(7, 31, 1.0, 4.0, -3.0);
  for (unsigned int y = 0; y < 31; ++y)
    for (unsigned int x = 0; x < 7; ++x) map.setCost(x, y, 0);
  planner.setStartAndGoal(pose(4.5, -2.5), pose(10.5, 27.5));
  require(planner.generatePath(path), "resized rolling window must remain usable");
  nav2_costmap_2d::Costmap2D single(1, 1, 1.0, 0.0, 0.0, 0);
  theta_star::ThetaStar one;
  one.costmap_ = &single;
  one.setStartAndGoal(pose(0.5, 0.5), pose(0.5, 0.5));
  require(one.generatePath(path), "one-cell map must succeed");
  std::cout << "PASS: isolated/unreachable maps, 100 failure/recovery cycles, invalid endpoints, same-cell paths, border LOS and resized windows\n";
}
