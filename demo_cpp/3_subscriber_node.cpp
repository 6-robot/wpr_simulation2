#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

std::shared_ptr<rclcpp::Node> node;

void Callback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(),"Receive : %s", msg->data.c_str());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

 node = std::make_shared<rclcpp::Node>("subscriber_node");

  auto subscriber= node->create_subscription<std_msgs::msg::String>("/my_topic", 10, &Callback);

  rclcpp::spin(node);
  
  rclcpp::shutdown();

  return 0;
}