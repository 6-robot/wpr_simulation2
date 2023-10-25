#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class WaypointNavigationNode : public rclcpp::Node
{
public:
  WaypointNavigationNode()
    : Node("waypoint_navigation_node")
  {
    navigation_pub_ = create_publisher<std_msgs::msg::String>("/waterplus/navi_waypoint", 10);

    result_sub_ = create_subscription<std_msgs::msg::String>(
      "/waterplus/navi_result", 10, std::bind(&WaypointNavigationNode::ResultCallback, this, std::placeholders::_1));

    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    std_msgs::msg::String waypoint_msg;
    waypoint_msg.data = "1";
    navigation_pub_->publish(waypoint_msg);
  }

private:
  void ResultCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "navi done")
    {
      RCLCPP_INFO(get_logger(), "Arrived !");
    }
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr result_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointNavigationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}