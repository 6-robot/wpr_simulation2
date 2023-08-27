#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
int nCount = 0;

void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    int nNum = msg->ranges.size();
    
    int nMid = nNum / 2;
    float fMidDist = msg->ranges[nMid];
    RCLCPP_INFO(node->get_logger(), "ranges[%d] = %f m", nMid, fMidDist);

    if (nCount > 0)
    {
        nCount--;
        return;
    }

    geometry_msgs::msg::Twist vel_msg;
    if (fMidDist < 1.5f)
    {
        vel_msg.angular.z = 0.3;
        nCount = 100;
    }
    else
    {
        vel_msg.linear.x = 0.1;
    }
    vel_pub->publish(vel_msg);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("lidar_behavior_node");

    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    auto lidar_sub = node->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, LidarCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}