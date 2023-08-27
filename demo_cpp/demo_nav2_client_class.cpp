#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Client : public rclcpp::Node
{
public:

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2Client()
  : Node("nav2_client_node")
  {

  }

  void send_goal(const double x, const double y, const double theta)
  {
    navigate_to_pose_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(shared_from_this(), "navigate_to_pose");

    if (!this->navigate_to_pose_client_ptr_->wait_for_action_server(std::chrono::seconds(10))) 
    {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      return;
    }

    auto goal_msg = NavigateToPose::Goal();

    goal_msg.pose.header.stamp = now();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.w = std::cos(theta * 0.5);
    goal_msg.pose.pose.orientation.z = std::sin(theta * 0.5);


    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_goal_options.feedback_callback =
      std::bind(&Nav2Client::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&Nav2Client::result_callback, this, _1);

    auto goal_handle_future = navigate_to_pose_client_ptr_->async_send_goal(
      goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "send_goal failed");
      return;
    }

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(
        get_logger(), "ExecutorClient: Execution was rejected by the action server");
      return;
    }
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_ptr_;

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaininf = %f",
      feedback->distance_remaining);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(get_logger(), "Success!!!");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(get_logger(), "Unknown result code");
        return;
    }
  }
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Nav2Client>();

  node->send_goal(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}