#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class TravelledPathPub : public rclcpp::Node
{
public:
    TravelledPathPub() : Node("travelled_path")
    {
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/travelled_path", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                latest_pose_.header = msg->header;
                latest_pose_.pose = msg->pose.pose;
            });

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            [this]() {
                if (latest_pose_.header.stamp.sec != 0) {  // check if initialized
                    path_.header.frame_id = "odom";
                    path_.poses.push_back(latest_pose_);
                    path_pub_->publish(path_);
                }
            });
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Path path_;
    geometry_msgs::msg::PoseStamped latest_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TravelledPathPub>());
    rclcpp::shutdown();
    return 0;
}