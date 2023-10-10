#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class IMUDataNode : public rclcpp::Node
{
public:
  IMUDataNode()
    : Node("imu_data_node")
  {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 
      100, 
      std::bind(&IMUDataNode::imuCallback, this, std::placeholders::_1));
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setX(msg->orientation.x);
    tf2_quaternion.setY(msg->orientation.y);
    tf2_quaternion.setZ(msg->orientation.z);
    tf2_quaternion.setW(msg->orientation.w);

    tf2::Matrix3x3 matrix(tf2_quaternion);

    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;
    RCLCPP_INFO(get_logger(), "roll= %.0f pitch= %.0f yaw= %.0f", roll, pitch, yaw);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMUDataNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}