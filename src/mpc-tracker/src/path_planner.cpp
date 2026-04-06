#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node
{
public:
  PathPublisher()
  : Node("path_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    auto timer_callback =
      [this]() -> void {
        auto path = nav_msgs::msg::Path();
        // Fill stamp
        path.header.stamp = this->now();
        path.header.frame_id = "odom";
        for (int i = 0; i < 20; ++i) {
            auto pose = geometry_msgs::msg::PoseStamped();
            // Fill stamp
            pose.header.stamp = this->now();
            pose.header.frame_id = "odom";
            // Fill Target Pose
            pose.pose.position.x = i * 0.5;
            pose.pose.position.y = i * 0.2;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        this->publisher_->publish(path);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}