#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>

class LidarBehaviorNode : public rclcpp::Node
{
public:
  LidarBehaviorNode()
    : Node("lidar_behavior_node")
  {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LidarBehaviorNode::LidarCallback, this, std::placeholders::_1));
  }

private:
  void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int nNum = msg->ranges.size();
    int nMid = nNum / 2;
    float fMidDist = msg->ranges[nMid];
    RCLCPP_INFO(get_logger(), "ranges[%d] = %f m", nMid, fMidDist);

    if (nCount_ > 0)
    {
        nCount_--;
        return;
    }

    geometry_msgs::msg::Twist vel_msg;
    if (fMidDist < 1.5f)
    {
        vel_msg.angular.z = 0.3;
        nCount_ = 100;
    }
    else
    {
        vel_msg.linear.x = 0.1;
    }
    vel_pub_->publish(vel_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  int nCount_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarBehaviorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}