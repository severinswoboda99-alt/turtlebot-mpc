#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class TravelledPathPub : public rclcpp::Node
{
public:
    TravelledPathPub() : Node("travelled_path")
    {
        auto odom_callback =
        [this](nav_msgs::msg::Odometry::SharedPtr msg) -> void {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg->header;
            path_.header.frame_id = "odom";
            pose.pose = msg->pose.pose;
            path_.poses.push_back(pose);
            path_pub_->publish(path_);
        };
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, odom_callback);

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/travelled_path", 10);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TravelledPathPub>());
    rclcpp::shutdown();
    return 0;
}