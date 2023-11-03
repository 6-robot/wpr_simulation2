#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/joint_state.hpp"

class ManiCtrlNode : public rclcpp::Node
{

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mani_pub_;
  sensor_msgs::msg::JointState mani_msg_;
  rclcpp::TimerBase::SharedPtr timer_;
  int pose_id_;

public:
  ManiCtrlNode() : Node("mani_ctrl_node")
  {
    mani_msg_.name.resize(2);
    mani_msg_.name[0] = "lift";
    mani_msg_.name[1] = "gripper";

    mani_msg_.position.resize(2);
    mani_msg_.position[0] = 0.0;
    mani_msg_.position[1] = 0.0;

    pose_id_ = 1;

    mani_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/wpb_home/mani_ctrl", 
      10
    );
    
    timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&ManiCtrlNode::TimerCallback,this)
    );
  }

  void TimerCallback()
  {
    if(pose_id_ == 1)
    {
      RCLCPP_WARN(this->get_logger(), "Pose 1");
      mani_msg_.position[0] = 0.0;
      mani_msg_.position[1] = 0.01;
      mani_pub_->publish(mani_msg_);
      pose_id_ = 2;
      return;
    }
 
    if(pose_id_ == 2)
    {
      RCLCPP_WARN(this->get_logger(), "Pose 2");
      mani_msg_.position[0] = 1.0;
      mani_msg_.position[1] = 0.1;
      mani_pub_->publish(mani_msg_);
      pose_id_ = 1;
      return;
    }
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