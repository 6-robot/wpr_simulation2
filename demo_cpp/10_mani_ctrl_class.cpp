#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

class ManiCtrlNode : public rclcpp::Node
{

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  sensor_msgs::msg::JointState mani_msg_;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  ManiCtrlNode() : Node("mani_ctrl_node")
  {
    mani_msg_.name.resize(2);
    mani_msg_.name[0] = "lift";
    mani_msg_.name[1] = "gripper";

    mani_msg_.position.resize(2);
    mani_msg_.position[0] = 0.0;
    mani_msg_.position[1] = 0.0;

    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/wpb_home/mani_ctrl", 10);
    timer_ = this->create_wall_timer(std::chrono::seconds(1),std::bind(&ManiCtrlNode::TimerCallback,this));
  }

  void TimerCallback()
  {
    mani_msg_.position[0] += 0.03;
    publisher_->publish(mani_msg_);
  }

};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto mani_ctrl_node = std::make_shared<ManiCtrlNode>();
  rclcpp::spin(mani_ctrl_node);
  rclcpp::shutdown();
  return 0;
}