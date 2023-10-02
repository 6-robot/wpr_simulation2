#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class PublisherNode : public rclcpp::Node
{
public:
  PublisherNode()
    : Node("publisher_node")
  {
    publisher_ = create_publisher<std_msgs::msg::String>("/my_topic", 10);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000), 
      std::bind(&PublisherNode::publishMessage, this)
      );
  }

private:
  void publishMessage()
  {
    message_.data = "Hello World!";
    publisher_->publish(message_);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  std_msgs::msg::String message_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}