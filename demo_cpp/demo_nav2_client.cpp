#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("nav2_client_node");

  auto navigation_client_ = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");

  if (!navigation_client_->wait_for_action_server(std::chrono::seconds(10))) 
  {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    return 1;
  }
  
  auto goal_msg = NavigateToPose::Goal();

  goal_msg.pose.header.stamp = node->get_clock()->now();
  goal_msg.pose.header.frame_id = "map";

  goal_msg.pose.pose.position.x = 3.0;
  goal_msg.pose.pose.position.y = 2.0;
  double theta = 1.57;
  goal_msg.pose.pose.orientation.w = std::cos(theta * 0.5);
  goal_msg.pose.pose.orientation.z = std::sin(theta * 0.5);


  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  auto goal_handle_future = navigation_client_->async_send_goal(goal_msg, send_goal_options);

  if (rclcpp::spin_until_future_complete(node, goal_handle_future) ==rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(node->get_logger(), "Arrived at WayPoint !");
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Failed to get to WayPoint ...");
  }

  rclcpp::shutdown();

  return 0;
}