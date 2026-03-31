#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

// Function to guarantee the angles are between pi and -pi
// double angle_wrap(double alpha){
//   while (alpha <= -M_PI)
//   {
//     alpha = alpha + 2 * M_PI;
//   }
//   while (alpha > M_PI)
//   {
//     alpha = alpha - 2 * M_PI;
//   }
//   return alpha;
// }

double calc_placeholder(){
  double r = 0;
  return r;
}

Eigen::Martrix<double,1,1> calc_hessian(Eigen::Martrix<double,1,1> Q, Eigen::Martrix<double,1,1> R, Eigen::Martrix<double,1,1> P, double delta_t, double theta_ref[], double v_ref[]){
  Eigen::Martrix<double,1,1> H << 0, 0;
  return H;
}

// ROS2 Node for the motor controller
class MPCNode : public rclcpp::Node {
public:
  MPCNode() : rclcpp::Node("mpc_node") {

    // Create Subscriber for current pose of TurtleBot
    auto odom_callback = [this](const nav_msgs::msg::Odometry & msg){
      this->x = msg.pose.pose.position.x;
      this->y = msg.pose.pose.position.y;
      double qx = msg.pose.pose.orientation.x;
      double qy = msg.pose.pose.orientation.y;
      double qz = msg.pose.pose.orientation.z;
      double qw = msg.pose.pose.orientation.w;
      this->theta = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)); // Conversion, quaternion to radian
    };
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, odom_callback);

    

    // Create Publisher for the TurtleBot3 movement topic
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      auto r = calc_placeholder();
      auto message = geometry_msgs::msg::TwistStamped();
      message.twist.linear.x = r;
      message.twist.angular.z = r;
      RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f angular.z=%.2f",
                  message.twist.linear.x, message.twist.angular.z);
      pub_->publish(message);
    });
  }

private:
    double x, y, theta, delta_t;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Main function to run the node
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCNode>());
  rclcpp::shutdown();
  return 0;
}