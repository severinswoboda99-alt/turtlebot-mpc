#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>

#include "mpc-tracker/spline/src/spline.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

using namespace std::chrono_literals;

class PathPublisher : public rclcpp::Node
{
public:
  PathPublisher() : Node("path_publisher") {
    // Initialize map index
    this->declare_parameter<int>("map", 0);
    map = this->get_parameter("map").as_int();
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PathPublisher::on_param_set, this, std::placeholders::_1));

    // Create map publisher
    publisher_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    auto timer_callback = [this]() -> void {
      auto path = nav_msgs::msg::Path();
      // Fill stamp
      auto now = this->now();
      path.header.stamp = now;
      path.header.frame_id = "odom";

      // Create map according to index
      switch (map) {
      case 0: {
        // Straight Line, forward, 3 meters, in 5 cm increments
        for (int i = 0; i < 60; ++i) {
          double x = i * 0.05;
          auto pose = make_pose(x, 0, 0, now);
          path.poses.push_back(pose); 
        } 
        break;
      }
      case 1: {
        // Spline "Wide Curve"
        std::vector<double> X = {0, 0.5, 1, 1.5, 2, 2.5, 3};
        std::vector<double> Y = {0, 0.1, 0.4, 0.75, 1.1, 1.4, 1.5};

        // default cubic spline (C^2) with natural boundary conditions (f''=0)
        tk::spline s(X,Y);			// X needs to be strictly increasing
        for (int i = 0; i < 100; ++i) {
          double x = (3.0 / 100.0) * i;
          double y = s((3.0 / 100.0) * i);
          double dydx = s.deriv(1, x);        // first derivative dy/dy
          double yaw  = std::atan2(dydx, 1.0);
          auto pose = make_pose(x, y, yaw, now);
          path.poses.push_back(pose);
        }
        break;
      }
      case 2: {
        // Spline "Reverse S"
        std::vector<double> X = {0, 0.5, 1, 1.5, 2, 2.5, 3, 3.5, 4};
        std::vector<double> Y = {0, 0.4, 0.6, 0.25, 0, -0.25, -0.6, -0.4, 0};

        // default cubic spline (C^2) with natural boundary conditions (f''=0)
        tk::spline s(X,Y);			// X needs to be strictly increasing
        for (int i = 0; i < 100; ++i) {
          double x = (4.0 / 100.0) * i;
          double y = s((4.0 / 100.0) * i);
          double dydx = s.deriv(1, x);        // first derivative dy/dy
          double yaw  = std::atan2(dydx, 1.0);
          auto pose = make_pose(x, y, yaw, now);
          path.poses.push_back(pose);
        }
        break;
      }
      case 3: {
        // Spline "Tight S"
        std::vector<double> X = {0, 0.5, 1, 1.5, 2, 2.5, 3};
        std::vector<double> Y = {0, 0.5, 0.25, -0.5, -0.25, 0.5, 0};

        // default cubic spline (C^2) with natural boundary conditions (f''=0)
        tk::spline s(X,Y);			// X needs to be strictly increasing
        for (int i = 0; i < 100; ++i) {
          double x = (3.0 / 100.0) * i;
          double y = s((3.0 / 100.0) * i);
          double dydx = s.deriv(1, x);        // first derivative dy/dy
          double yaw  = std::atan2(dydx, 1.0);
          auto pose = make_pose(x, y, yaw, now);
          path.poses.push_back(pose);
        }
        break;
      }
      default:
        break;
      }
      this->publisher_->publish(path);
    };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }

private:
  // Map Index
  // 0: Straight Line
  // 1: Spline "Wide Curve"
  // 2: Spline "Reverse S"
  // 3: Spline "Tight S"
  int map;

  // Function to create pose message from known data
  geometry_msgs::msg::PoseStamped make_pose(double x, double y, double yaw, rclcpp::Time now) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.header.stamp = now;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.z = sin(yaw / 2);
    pose.pose.orientation.w = cos(yaw / 2);
    return pose;
  }

  // Dynamic Reconfigure
  rcl_interfaces::msg::SetParametersResult
  on_param_set(const std::vector<rclcpp::Parameter> &params) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    for (const auto &param : params) {
    const std::string &name = param.get_name();

    if (name == "map") {
      map = param.as_int();
    } else {
      res.successful = false;
      res.reason = "Unknown parameter: " + name;
    }
  }
  return res;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathPublisher>());
  rclcpp::shutdown();
  return 0;
}