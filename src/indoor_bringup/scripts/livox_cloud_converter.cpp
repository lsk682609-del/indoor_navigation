#include <cmath>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class LivoxCloudConverter : public rclcpp::Node
{
public:
  LivoxCloudConverter() : Node("livox_cloud_converter")
  {
    const auto input = declare_parameter("input_topic", "/livox/lidar");
    const auto output = declare_parameter("output_topic", "/livox/lidar/pointcloud");
    frame_ = declare_parameter("sensor_frame", "livox_frame");
    publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(output, rclcpp::SensorDataQoS());
    subscription_ = create_subscription<livox_ros_driver2::msg::CustomMsg>(
      input, rclcpp::SensorDataQoS(),
      [this](livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {
        if (msg->header.frame_id != frame_ || msg->point_num != msg->points.size()) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Reject Livox cloud: sensor frame or point_num mismatch");
          return;
        }
        sensor_msgs::msg::PointCloud2 out;
        out.header = msg->header;  // Preserve measurement time and sensor coordinates.
        out.height = 1;
        out.is_dense = true;
        sensor_msgs::PointCloud2Modifier modifier(out);
        modifier.setPointCloud2Fields(4,
          "x", 1, sensor_msgs::msg::PointField::FLOAT32,
          "y", 1, sensor_msgs::msg::PointField::FLOAT32,
          "z", 1, sensor_msgs::msg::PointField::FLOAT32,
          "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        size_t count = 0;
        for (const auto & point : msg->points) {
          if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {++count;}
        }
        modifier.resize(count);
        sensor_msgs::PointCloud2Iterator<float> x(out, "x"), y(out, "y"), z(out, "z"), intensity(out, "intensity");
        for (const auto & point : msg->points) {
          if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {continue;}
          *x = point.x; *y = point.y; *z = point.z; *intensity = point.reflectivity;
          ++x; ++y; ++z; ++intensity;
        }
        publisher_->publish(out);
      });
  }
private:
  std::string frame_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxCloudConverter>());
  rclcpp::shutdown();
  return 0;
}
