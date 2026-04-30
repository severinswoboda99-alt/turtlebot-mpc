#include "aruco_markers/aruco_markers.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto aruco_node = std::make_shared<ArucoMarkersNode>();
  aruco_node->initialize();
  rclcpp::spin(aruco_node);
  rclcpp::shutdown();
  return 0;
}
