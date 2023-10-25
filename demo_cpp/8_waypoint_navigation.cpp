#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

std::shared_ptr<rclcpp::Node> node;

void ResultCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if(msg->data == "done")
    {
        RCLCPP_INFO(node->get_logger(), "Arrived !");
    }
}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("waypoint_navigation_node");

    auto navigation_pub = node->create_publisher<std_msgs::msg::String>(
    "/waterplus/navi_waypoint", 10);

    auto lidar_sub = node->create_subscription<std_msgs::msg::String>(
    "/waterplus/navi_result", 10, ResultCallback);

    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    std_msgs::msg::String waypoint_msg;
    waypoint_msg.data = "1";
    navigation_pub->publish(waypoint_msg);

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}