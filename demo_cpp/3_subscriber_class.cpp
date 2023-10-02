#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode()
    : Node("subscriber_node")
  {
    subscriber_ = create_subscription<std_msgs::msg::String>(
      "/my_topic", 
      10, 
      std::bind(&SubscriberNode::callback, this, std::placeholders::_1));
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Receive : %s", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SubscriberNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}