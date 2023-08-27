#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("publisher_node");

  auto publisher = node->create_publisher<std_msgs::msg::String>("/topic", 10);

  std_msgs::msg::String message;
  message.data = "Hello World!";

  rclcpp::Rate loop_rate(1);

  while (rclcpp::ok()) 
  {
    publisher->publish(message);
    loop_rate.sleep();
  }
  
  rclcpp::shutdown();

  return 0;
}