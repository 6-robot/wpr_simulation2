#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wpr_simulation2/msg/object.hpp>

class GrabObjectNode : public rclcpp::Node
{
public:
  GrabObjectNode()
    : Node("grab_object_node")
  {
    cmd_pub_ = create_publisher<std_msgs::msg::String>("/wpb_home/behavior", 10);
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    mani_pub_ = create_publisher<sensor_msgs::msg::JointState>("/wpb_home/mani_ctrl", 10);
    object_sub_ = create_subscription<wpr_simulation2::msg::Object>(
      "/wpb_home/objects_3d", 10, std::bind(&GrabObjectNode::objectCallback, this, std::placeholders::_1)
      );

    align_x_ = 1.0;
    align_y_ = 0.0;
    object_x_ = 0.0;
    object_y_ = 0.0;
    object_z_ = 0.0;
    grab_step_ = STEP_WAIT;

    timer_ = create_wall_timer(std::chrono::milliseconds(33), std::bind(&GrabObjectNode::timerCallback, this));
  }

private:
  void objectCallback(const wpr_simulation2::msg::Object::SharedPtr msg)
  {
    if (grab_step_ == STEP_WAIT)
    {
      grab_step_ = STEP_ALIGN_OBJ;
    }
    if (grab_step_ == STEP_ALIGN_OBJ)
    {
      object_x_ = msg->x[0];
      object_y_ = msg->y[0];
      object_z_ = msg->z[0];
    }
  }

  void timerCallback()
  {
    if (grab_step_ == STEP_WAIT)
    {
      std_msgs::msg::String start_msg;
      start_msg.data = "start objects";
      cmd_pub_->publish(start_msg);
      return;
    }
    if (grab_step_ == STEP_ALIGN_OBJ)
    {
      float diff_x = object_x_ - align_x_;
      float diff_y = object_y_ - align_y_;
      geometry_msgs::msg::Twist vel_msg;
      if (std::fabs(diff_x) > 0.02 || std::fabs(diff_y) > 0.01)
      {
        vel_msg.linear.x = diff_x * 0.8;
        vel_msg.linear.y = diff_y * 0.8;
      }
      else
      {
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        std_msgs::msg::String start_msg;
        start_msg.data = "stop objects";
        cmd_pub_->publish(start_msg);
        grab_step_ = STEP_HAND_UP;
      }
      RCLCPP_INFO(get_logger(), "[STEP_ALIGN_OBJ] vel = ( %.2f , %.2f )", vel_msg.linear.x, vel_msg.linear.y);
      vel_pub_->publish(vel_msg);
      return;
    }
    if (grab_step_ == STEP_HAND_UP)
    {
        RCLCPP_INFO(get_logger(), "[STEP_HAND_UP]");
        sensor_msgs::msg::JointState mani_msg;
        mani_msg.name.resize(2);
        mani_msg.name[0] = "lift";
        mani_msg.name[1] = "gripper";
        mani_msg.position.resize(2);
        mani_msg.position[0] = object_z_;
        mani_msg.position[1] = 0.15;
        mani_pub_->publish(mani_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(8000));
        grab_step_ = STEP_FORWARD;
        return;
    }
    if (grab_step_ == STEP_FORWARD)
    {
      RCLCPP_INFO(
                get_logger(), 
                "[STEP_FORWARD] object_x = %.2f", 
                object_x_
            );
              geometry_msgs::msg::Twist vel_msg;
              vel_msg.linear.x = 0.1;
              vel_msg.linear.y = 0.0;
              vel_pub_->publish(vel_msg);
              int forward_duration = (object_x_ - 0.65) * 20000;
              rclcpp::sleep_for(std::chrono::milliseconds(forward_duration));
      grab_step_ = STEP_GRAB;
      return;
    }
    if (grab_step_ == STEP_GRAB)
    {
        RCLCPP_INFO(get_logger(), "[STEP_GRAB]");
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_pub_->publish(vel_msg);
        sensor_msgs::msg::JointState mani_msg;
        mani_msg.name.resize(2);
        mani_msg.name[0] = "lift";
        mani_msg.name[1] = "gripper";
        mani_msg.position.resize(2);
        mani_msg.position[0] = object_z_;
        mani_msg.position[1] = 0.07;
        mani_pub_->publish(mani_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(5000));
        grab_step_ = STEP_OBJ_UP;
        return;
    }
    if (grab_step_ == STEP_OBJ_UP)
    {
        RCLCPP_INFO(get_logger(), "[STEP_OBJ_UP]");
        sensor_msgs::msg::JointState mani_msg;
        mani_msg.name.resize(2);
        mani_msg.name[0] = "lift";
        mani_msg.name[1] = "gripper";
        mani_msg.position.resize(2);
        mani_msg.position[0] = object_z_ + 0.05;
        mani_msg.position[1] = 0.07;
        mani_pub_->publish(mani_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(5000));
        grab_step_ = STEP_BACKWARD;
        return;
    }
    if (grab_step_ == STEP_BACKWARD)
    {
        RCLCPP_INFO(get_logger(), "[STEP_BACKWARD]");
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = -0.1;
        vel_msg.linear.y = 0;
        vel_pub_->publish(vel_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(10000));
        grab_step_ = STEP_DONE;
        RCLCPP_INFO(get_logger(), "[STEP_DONE]");
        return;
    }
    if (grab_step_ == STEP_DONE)
    {
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_pub_->publish(vel_msg);
        return;
    }
  }

  // Constants
  static constexpr int STEP_WAIT = 0;
  static constexpr int STEP_ALIGN_OBJ = 1;
  static constexpr int STEP_HAND_UP = 2;
  static constexpr int STEP_FORWARD = 3;
  static constexpr int STEP_GRAB = 4;
  static constexpr int STEP_OBJ_UP = 5;
  static constexpr int STEP_BACKWARD = 6;
  static constexpr int STEP_DONE = 7;

  // Member variables
  int grab_step_;
  float object_x_;
  float object_y_;
  float object_z_;
  float align_x_;
  float align_y_;

  // ROS publishers and subscribers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mani_pub_;
  rclcpp::Subscription<wpr_simulation2::msg::Object>::SharedPtr object_sub_;

  // Timer for control loop
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GrabObjectNode>();
 rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}