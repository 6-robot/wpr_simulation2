#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarDataNode : public rclcpp::Node
{
public:
  LidarDataNode()
    : Node("lidar_data_node")
  {
    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&LidarDataNode::LidarCallback, this, std::placeholders::_1));
  }

private:
  void LidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    int nNum = msg->ranges.size();
    int nMid = nNum / 2;
    float fMidDist = msg->ranges[nMid];
    RCLCPP_INFO(get_logger(), "ranges[%d] = %f m", nMid, fMidDist);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarDataNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}