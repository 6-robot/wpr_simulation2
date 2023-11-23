#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navigation_pub;
std::string dest_waypoint = "1";

void ResultCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if(msg->data == "navi done")
    {
        if(dest_waypoint == "1")
        {
            RCLCPP_INFO(node->get_logger(), "Arrived Waypoint 1 !");
            std_msgs::msg::String waypoint_msg;
            dest_waypoint = "2";
            waypoint_msg.data = dest_waypoint;
            navigation_pub->publish(waypoint_msg);
            return;
        }

        if(dest_waypoint == "2")
        {
            RCLCPP_INFO(node->get_logger(), "Arrived Waypoint 2 !");
            std_msgs::msg::String waypoint_msg;
            dest_waypoint = "3";
            waypoint_msg.data = dest_waypoint;
            navigation_pub->publish(waypoint_msg);
            return;
        }

        if(dest_waypoint == "3")
        {
            RCLCPP_INFO(node->get_logger(), "Arrived Waypoint 3 !");
            return;
        }
    }
}
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("waypoint_navigation_node");

    navigation_pub = node->create_publisher<std_msgs::msg::String>(
    "/waterplus/navi_waypoint", 10);

    auto result_sub = node->create_subscription<std_msgs::msg::String>(
    "/waterplus/navi_result", 10, ResultCallback);

    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    std_msgs::msg::String waypoint_msg;
    dest_waypoint = "1";
    waypoint_msg.data = dest_waypoint;
    navigation_pub->publish(waypoint_msg);

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}