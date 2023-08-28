#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("waypoint_navigation_node");

  auto navigation_pub = node->create_publisher<std_msgs::msg::String>("/waterplus/navi_waypoint", 10);

  rclcpp::sleep_for(std::chrono::milliseconds(1000));

  std_msgs::msg::String waypoint_msg;
  waypoint_msg.data = "1";

  navigation_pub->publish(waypoint_msg);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}