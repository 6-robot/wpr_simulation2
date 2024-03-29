#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

std::shared_ptr<rclcpp::Node> node;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("mani_ctrl_node");

  auto mani_pub = node->create_publisher<sensor_msgs::msg::JointState>(
    "/wpb_home/mani_ctrl", 
    10
  );

  sensor_msgs::msg::JointState mani_msg;

  mani_msg.name.resize(2);
  mani_msg.name[0] = "lift";
  mani_msg.name[1] = "gripper";
  
  mani_msg.position.resize(2);
  mani_msg.position[0] = 0.0;
  mani_msg.position[1] = 0.0;

  rclcpp::Rate loop_rate(0.1);

  while (rclcpp::ok()) 
  {
    RCLCPP_WARN(node->get_logger(), "Pose 1");
    mani_msg.position[0] = 0.0;
    mani_msg.position[1] = 0.01;
    mani_pub->publish(mani_msg);
    loop_rate.sleep();

    RCLCPP_WARN(node->get_logger(), "Pose 2");
    mani_msg.position[0] = 1.0;
    mani_msg.position[1] = 0.1;
    mani_pub->publish(mani_msg);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}