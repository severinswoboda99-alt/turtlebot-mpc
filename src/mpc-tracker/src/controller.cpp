#include <chrono>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

// Function to guarantee the angles are between pi and -pi
double angle_wrap(double alpha){
  while (alpha <= -M_PI)
  {
    alpha = alpha + 2 * M_PI;
  }
  while (alpha > M_PI)
  {
    alpha = alpha - 2 * M_PI;
  }
  return alpha;
}

// Function to calculate the linear and angular velocity
static std::pair<double, double> calculate_velocity(double x_goal, double y_goal, double theta_goal, double x, double y, 
                                double theta, double k_rho, double k_alpha, double k_beta, bool start){
    // Define return variable
    std::pair<double, double>  r;
    
    // Calculate pose difference
    double x_d = x_goal - x;
    double y_d = y_goal - y;
    double theta_d = angle_wrap(theta_goal - theta);

    if(start){  // Only drive the robot if the start parameter is active
      if(abs(x_d) > 0.02 || abs(y_d) > 0.02){     // If robot is not at goal position calculate velocity normally
        // Calculate rho
        double rho = sqrt(x_d * x_d + y_d * y_d);
        double alpha = angle_wrap(-theta + atan2(y_d,x_d));
        double beta = angle_wrap(-theta_d - alpha);

        // Calculate motion commands 𝒗 and w
        r.first = k_rho * rho;                          // Linear Velocity
        r.second = k_alpha * alpha + k_beta * beta;     // Angular Velocity
      }
      else if(abs(theta_d) > 0.01){               // If robot is at target position but orientation is off calculate heading based on angle error alone;
        r.first = 0.0;                            // Otherwise function becomes unstable and robot spins in place
        r.second = - k_beta * theta_d;
      }
    }
    else{
      r.first = 0.0;
      r.second = 0.0;
    }
    return r;
}

// ROS2 Node for the motor controller
class MotorControllerNode : public rclcpp::Node {
public:
  MotorControllerNode() : rclcpp::Node("linear_controller") {

    // Declare Parameters for Dynamic Reconfigure and initial variable values
    this->declare_parameter<double>("k_rho", 0.15);
    this->declare_parameter<double>("k_alpha", 0.8);
    this->declare_parameter<double>("k_beta", -0.4);
    this->declare_parameter<double>("x_goal", 0.0);
    this->declare_parameter<double>("y_goal", 0.0);
    this->declare_parameter<double>("theta_goal", 0.0);
    this->declare_parameter<bool>("start", false);

    k_rho = this->get_parameter("k_rho").as_double();
    k_alpha = this->get_parameter("k_alpha").as_double();
    k_beta = this->get_parameter("k_beta").as_double();
    x_goal = this->get_parameter("x_goal").as_double();
    y_goal = this->get_parameter("y_goal").as_double();
    theta_goal = this->get_parameter("theta_goal").as_double();
    start = this->get_parameter("start").as_bool();

    x = 0.0;
    y = 0.0;
    theta = 0.0;

    // Register parameter-change callback (live updates)
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MotorControllerNode::on_param_set, this, std::placeholders::_1));

    // Create Subscriber for current pose of TurtleBot
    auto odom_callback = [this](const nav_msgs::msg::Odometry & msg){
      this->x = msg.pose.pose.position.x;
      this->y = msg.pose.pose.position.y;
      double qx = msg.pose.pose.orientation.x;
      double qy = msg.pose.pose.orientation.y;
      double qz = msg.pose.pose.orientation.z;
      double qw = msg.pose.pose.orientation.w;
      this->theta = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)); // Odometry data is in quaternions, so they are transformed to radian
    };
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, odom_callback);

    // Create Publisher for the TurtleBot3 movement topic
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      auto r = calculate_velocity(x_goal, y_goal, theta_goal, x, y, theta, k_rho, k_alpha, k_beta, start);
      auto message = geometry_msgs::msg::TwistStamped();
      message.twist.linear.x = r.first;
      message.twist.angular.z = r.second;
      RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f angular.z=%.2f",
                  message.twist.linear.x, message.twist.angular.z);
      pub_->publish(message);
    });
  }

private:
    double k_rho, k_alpha, k_beta;
    double x_goal, y_goal, theta_goal;
    double x, y, theta;
    bool start;

    rcl_interfaces::msg::SetParametersResult
    on_param_set(const std::vector<rclcpp::Parameter> &params) {
      rcl_interfaces::msg::SetParametersResult res;
      res.successful = true;

      for (const auto &param : params) {
      const std::string &name = param.get_name();

      if (name == "k_rho") {
        k_rho = param.as_double();
      } else if (name == "k_alpha") {
        k_alpha = param.as_double();
      } else if (name == "k_beta") {
        k_beta = param.as_double();
      } else if (name == "x_goal") {
        x_goal = param.as_double();
      } else if (name == "y_goal") {
        y_goal = param.as_double();
      } else if (name == "theta_goal") {
        theta_goal = param.as_double();
      } else if (name == "start") {
        start = param.as_bool();
      } else {
        res.successful = false;
        res.reason = "Unknown parameter: " + name;
      }
    }
    return res;
  }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

};

// Main function to run the node
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControllerNode>());
  rclcpp::shutdown();
  return 0;
}