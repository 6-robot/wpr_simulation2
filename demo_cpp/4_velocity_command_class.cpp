#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class VelocityCommandNode : public rclcpp::Node
{
public:
  VelocityCommandNode()
    : Node("velocity_command_node")
  {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    vel_msg_.linear.x = 0.1;
    vel_msg_.linear.y = 0.0;
    vel_msg_.linear.z = 0.0;
    vel_msg_.angular.x = 0.0;
    vel_msg_.angular.y = 0.0;
    vel_msg_.angular.z = 0.0;

    timer_ = create_wall_timer(
      std::chrono::milliseconds(33), 
      std::bind(&VelocityCommandNode::publishVelocity, 
      this));
  }

private:
  void publishVelocity()
  {
    vel_pub_->publish(vel_msg_);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  geometry_msgs::msg::Twist vel_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VelocityCommandNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}