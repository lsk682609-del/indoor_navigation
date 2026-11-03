#include <cmath>
#include <iostream>
#include <stdexcept>
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "nav2_a_star_rpp/regulated_pure_pursuit_controller.hpp"

void require(bool value, const char * message)
{
  if (!value) {throw std::runtime_error(message);}
}

class ControllerProbe : public nav2_a_star_rpp::RegulatedPurePursuitController
{
public:
  explicit ControllerProbe(std::shared_ptr<nav2_costmap_2d::Costmap2DROS> map)
  {
    costmap_ros_ = map;
    costmap_ = map->getCostmap();
    planner_ = std::make_unique<theta_star::ThetaStar>();
    planner_->lethal_cost_ = 140;
    desired_linear_vel_ = 1.0;
    front_stop_clearance_ = 0.35;
    front_slow_clearance_ = 1.2;
    max_linear_accel_ = 0.5;
    max_linear_decel_ = 1.0;
    control_duration_ = 0.05;
    use_rotate_to_heading_ = true;
    rotate_to_heading_min_angle_ = 0.5;
    avoid_flag_ = false;
    clock_ = map->get_clock();
    tf_ = std::make_shared<tf2_ros::Buffer>(clock_);
    tf_->setUsingDedicatedThread(true);  // This fixture only uses the identity map->map transform.
    max_allowed_time_to_collision_up_to_carrot_ = 1.5;
    carrot_arc_pub_ = map->create_publisher<nav_msgs::msg::Path>("door_test_arc", 1);
    carrot_arc_pub_->on_activate();
    door_min_width_ = 0.62;
    door_max_width_ = 1.2;
    door_cross_speed_ = 0.25;
    polyline_collision_check_resolution_ = 0.025;
    collision_checker_ = std::make_unique<
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);
  }
  using RegulatedPurePursuitController::isPolylineSegmentFree;
  using RegulatedPurePursuitController::isSmoothingPoseFree;
  bool rotate(double angle, double clearance, bool avoiding = true)
  {
    avoid_flag_ = avoiding;
    geometry_msgs::msg::PoseStamped carrot;
    carrot.pose.position.x = std::cos(angle);
    carrot.pose.position.y = std::sin(angle);
    double error;
    return shouldRotateToPath(carrot, error, clearance);
  }
  double stepVelocity(double target, double clearance)
  {
    last_linear_command_ = limitForwardVelocity(target, clearance);
    return last_linear_command_;
  }
  void seedTemporaryDetour(double detour_end, double final_end, bool with_tail)
  {
    avoid_flag_ = true;
    global_plan_.header.frame_id = track_path_.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped end;
    end.pose.position.x = detour_end;
    track_path_.poses = {end};
    end.pose.position.x = final_end;
    global_plan_.poses = {end};
    join_idx_ = with_tail ? 0 : 1;
    global_path_tail_start_idx_ = with_tail ? 1 : 0;
  }
  using RegulatedPurePursuitController::shouldExtendDetour;
  using RegulatedPurePursuitController::isCollisionImminent;
  using RegulatedPurePursuitController::footprintCostAt;
  using RegulatedPurePursuitController::frontClearDistance;
  using RegulatedPurePursuitController::rearClearDistance;
  int doorPhaseFor(double width, double route_y = 0.0, bool stale = false)
  {
    resetDoorPhase();
    global_plan_.header.frame_id = "map";
    geometry_msgs::msg::PoseStamped start, end;
    start.pose.position.x = 2.0;
    start.pose.position.y = end.pose.position.y = 2.0 + route_y;
    end.pose.position.x = 4.5;
    global_plan_.poses = {start, end};
    auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    cloud->header.frame_id = "map";
    cloud->header.stamp = clock_->now() - rclcpp::Duration::from_seconds(stale ? 1.0 : 0.0);
    sensor_msgs::PointCloud2Modifier modifier(*cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(80);
    sensor_msgs::PointCloud2Iterator<float> x(*cloud,"x"), y(*cloud,"y"), z(*cloud,"z");
    for (int side : {-1, 1}) {
      for (int bin = 0; bin < 4; ++bin) {
        for (int point = 0; point < 10; ++point, ++x, ++y, ++z) {
          *x = 3.0 + 0.01 * point;
          *y = 2.0 + side * (width / 2 + bin * 0.05);
          *z = 0.5 + 0.03 * point;
        }
      }
    }
    doorCloudCallback(cloud);
    geometry_msgs::msg::PoseStamped robot;
    robot.header.frame_id = "map"; robot.pose.orientation.w = 1.0;
    robot.pose.position.x = robot.pose.position.y = 2.0;
    updateDoorPhase(robot, geometry_msgs::msg::Twist{});
    updateDoorPhase(robot, geometry_msgs::msg::Twist{});
    return static_cast<int>(door_phase_);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  {
    auto map_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("detour_test_costmap");
    map_ros->set_parameter(rclcpp::Parameter("plugins", std::vector<std::string>{}));
    map_ros->set_parameter(rclcpp::Parameter("footprint_padding", 0.02));
    map_ros->on_configure(rclcpp_lifecycle::State());
    std::vector<geometry_msgs::msg::Point> footprint;
    for (auto xy : std::vector<std::pair<double, double>>{
        {0.275, 0.23}, {0.275, -0.23}, {-0.23, -0.23}, {-0.23, 0.23}})
    {
      geometry_msgs::msg::Point p;
      p.x = xy.first; p.y = xy.second;
      footprint.push_back(p);
    }
    map_ros->setRobotFootprint(footprint);
    auto map = map_ros->getCostmap();
    map->resizeMap(500, 500, 0.01, 0.0, 0.0);
    ControllerProbe probe(map_ros);
    auto reset = [&]() {
        for (unsigned int y = 0; y < 500; ++y)
          for (unsigned int x = 0; x < 500; ++x) {map->setCost(x, y, 0);}
      };
    reset();
    // A soft-cost change may preserve a safe detour, but cannot become a new plan.
    for (unsigned int x = 200; x <= 300; ++x) {map->setCost(x, 200, 180);}
    require(!probe.isPolylineSegmentFree(2, 2, 3, 2), "planning buffer must be enforced");
    require(probe.isPolylineSegmentFree(2, 2, 3, 2, 253), "soft costs must not churn a safe detour");
    map->setCost(250, 200, 254);
    require(!probe.isPolylineSegmentFree(2, 2, 3, 2, 253), "new obstacle must invalidate detour");
    reset();
    // Center remains free while the front corner overlaps; rotating changes the footprint.
    for (unsigned int y = 223; y <= 227; ++y)
      for (unsigned int x = 228; x <= 231; ++x) {map->setCost(x, y, 254);}
    require(!probe.isSmoothingPoseFree(2, 2, 0), "front corner collision must reject smoothing");
    require(probe.isSmoothingPoseFree(2, 2, M_PI / 2), "footprint must use candidate yaw");
    require(!probe.isSmoothingPoseFree(0.1, 0.1, 0), "off-map footprint must be rejected");
    reset();
    map->setCost(200, 200, 255);
    require(!probe.isSmoothingPoseFree(2, 2, 0), "unknown center must be rejected");
    // Persistent partial clearance must converge to its cap instead of a crawl.
    double velocity = 0.0;
    for (int tick = 0; tick < 80; ++tick) {
      const double previous = velocity;
      velocity = probe.stepVelocity(1.0, 0.775);
      require(velocity - previous <= 0.0250001, "linear acceleration limit exceeded");
    }
    require(std::abs(velocity - 0.5) < 1e-6, "clearance cap must not compound over cycles");
    for (int tick = 0; tick < 30; ++tick) {velocity = probe.stepVelocity(0.2, 0.775);}
    require(std::abs(velocity - 0.2) < 1e-6, "curvature cap must remain active");
    require(!probe.rotate(0.45, 0.9), "moderate detour angle must allow moving turns");
    require(probe.rotate(0.6, 0.9), "large heading error must align first");
    require(probe.rotate(0.48, 0.9), "rotation must retain state below entry threshold");
    require(probe.rotate(0.12, 0.9), "rotation must continue until aligned");
    require(!probe.rotate(0.08, 0.9), "alignment must finish without waiting");
    for (int tick = 0; tick < 80; ++tick) {
      require(!probe.rotate(tick % 2 ? 0.121 : 0.119, 0.9), "small-angle jitter must not stop motion");
    }
    require(probe.rotate(0.30, 0.4), "near-stop obstacle must still trigger early alignment");
    require(!probe.rotate(0.08, 0.4), "near-stop alignment must release once aligned");
    geometry_msgs::msg::PoseStamped robot;
    robot.header.frame_id = "map";
    robot.pose.position.x = 4.5;
    probe.seedTemporaryDetour(5.0, 9.0, false);
    require(probe.shouldExtendDetour(robot), "temporary detour must extend before end braking");
    probe.seedTemporaryDetour(5.0, 5.0, false);
    require(!probe.shouldExtendDetour(robot), "final goal must retain approach braking");
    probe.seedTemporaryDetour(5.0, 9.0, true);
    require(!probe.shouldExtendDetour(robot), "complete detour must retain its tail");
    reset();
    require(probe.doorPhaseFor(0.70) == 3, "0.70 m observed on-route doorway must enter CROSS");
    require(probe.doorPhaseFor(0.45) == 0, "undersized doorway must not enter servo");
    require(probe.doorPhaseFor(0.70, 2.0) == 0, "off-route doorway must not take control");
    require(probe.doorPhaseFor(0.70, 0.0, true) == 0, "stale lidar must not start servo");
    // Circumscribed inflation can mark the entire floor 253 in a usable 0.70 m gap.
    for (unsigned int y = 0; y < 500; ++y)
      for (unsigned int x = 0; x < 500; ++x) {map->setCost(x, y, 253);}
    for (unsigned int x = 100; x < 400; ++x) {
      map->setCost(x, 165, 254); map->setCost(x, 235, 254);
    }
    robot.pose.position.x = robot.pose.position.y = 2.0;robot.pose.orientation.w = 1.0;
    require(probe.footprintCostAt(2,2,0) < 254, "real footprint must fit 0.70 m gap despite 253 floor");
    require(!probe.isCollisionImminent(robot,0.25,0.0,0.75), "safe narrow crossing must be allowed");
    for (unsigned int y = 165; y <= 235; ++y) {map->setCost(260,y,254);}
    require(probe.isCollisionImminent(robot,0.25,0.0,0.75), "new obstacle must stop narrow crossing");
    // Switching vehicle dimensions must move clearance origins with the body,
    // rather than retaining the simulator's fixed front/rear overhangs.
    reset();
    std::vector<geometry_msgs::msg::Point> large_footprint;
    for (auto xy : std::vector<std::pair<double, double>>{
        {0.5,0.35}, {0.5,-0.35}, {-0.6,-0.35}, {-0.6,0.35}})
    {
      geometry_msgs::msg::Point point;
      point.x = xy.first; point.y = xy.second;
      large_footprint.push_back(point);
    }
    map_ros->setRobotFootprint(large_footprint);
    for (unsigned int y = 140; y <= 260; ++y) {
      for (unsigned int x = 300; x <= 305; ++x) {map->setCost(x,y,254);}
      for (unsigned int x = 100; x <= 110; ++x) {map->setCost(x,y,254);}
    }
    const double front_clear = probe.frontClearDistance(robot);
    const double rear_clear = probe.rearClearDistance(robot, 1.0);
    require(front_clear > 0.4 && front_clear < 0.56, "front clearance must follow real footprint");
    require(rear_clear > 0.15 && rear_clear < 0.4, "rear clearance must follow real footprint");
    map_ros->on_cleanup(rclcpp_lifecycle::State());
  }
  rclcpp::shutdown();
  std::cout << "PASS: buffered planning, stable detour reuse, new obstacle rejection, yaw-aware padded corners, map bounds and unknown cells\n";
  std::cout << "PASS: no compounded clearance scaling, acceleration and curve caps, rotation hysteresis, small-angle jitter and temporary detour continuation\n";
  std::cout << "PASS: 0.70 m door perception, undersized/off-route/stale rejection, 253 floor crossing and physical obstacle prediction\n";
  std::cout << "PASS: changed vehicle footprint updates front and rear clearance origins\n";
}
