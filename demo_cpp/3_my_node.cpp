#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("my_node");

  RCLCPP_INFO(node->get_logger(), "Hello world!");
  while (rclcpp::ok()) 
  {
    ;
  }
  
  rclcpp::shutdown();

  return 0;
}