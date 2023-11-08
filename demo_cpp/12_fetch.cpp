#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#define STEP_WAIT           0
#define STEP_GOTO_KITCHEN   1
#define STEP_GRAB_DRINK     2
#define STEP_GOTO_GUEST     3
#define STEP_DONE           4
static int fetch_step = STEP_WAIT;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr navi_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_pub;

void NaviResultCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "[NaviResultCallback] %s",msg->data.c_str());
    if(fetch_step == STEP_GOTO_KITCHEN && msg->data == "navi done")
    {
        std_msgs::msg::String msg;
        msg.data = "start grab";
        behavior_pub->publish(msg);
        fetch_step = STEP_GRAB_DRINK;
        RCLCPP_INFO(node->get_logger(), "[STEP_GOTO_KITCHEN] -> [STEP_GRAB_DRINK]");
    }

    if(fetch_step == STEP_GOTO_GUEST && msg->data == "navi done")
    {
        fetch_step = STEP_DONE;
        RCLCPP_INFO(node->get_logger(), "[STEP_GOTO_GUEST] -> [STEP_DONE]");
    }
}

void GrabResultCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "[GrabResultCallback] %s",msg->data.c_str());
    if(fetch_step == STEP_GRAB_DRINK && msg->data == "grab done")
    {
        std_msgs::msg::String msg;
        msg.data = "guest";
        navi_pub->publish(msg);
        fetch_step = STEP_GOTO_GUEST;
        RCLCPP_INFO(node->get_logger(), "[STEP_GRAB_DRINK] -> [STEP_GOTO_GUEST]");
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("fetch_node");

    navi_pub = node->create_publisher<std_msgs::msg::String>(
        "/waterplus/navi_waypoint", 
        10
    );
    behavior_pub = node->create_publisher<std_msgs::msg::String>(
        "/wpb_home/behavior", 
        10
    );
    auto navi_result_sub = node->create_subscription<std_msgs::msg::String>(
        "waterplus/navi_result", 
        10, 
        NaviResultCallback
    );
    auto grab_result_sub = node->create_subscription<std_msgs::msg::String>(
        "/wpb_home/grab_result", 
        10, 
        GrabResultCallback
    );
    
    rclcpp::sleep_for(std::chrono::milliseconds(1000));

    rclcpp::Rate loop_rate(30);

    while(rclcpp::ok())
    {
        if(fetch_step == STEP_WAIT)
        {
            std_msgs::msg::String msg;
            msg.data = "kitchen";
            navi_pub->publish(msg);
            fetch_step = STEP_GOTO_KITCHEN;
            RCLCPP_INFO(node->get_logger(), "[STEP_WAIT] -> [STEP_GOTO_KITCHEN]");
        }
        
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}