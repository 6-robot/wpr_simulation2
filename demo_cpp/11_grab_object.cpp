#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wpr_simulation2/msg/object.hpp>

#define STEP_WAIT           0
#define STEP_ALIGN_OBJ      1
#define STEP_HAND_UP        2
#define STEP_FORWARD        3
#define STEP_GRAB           4
#define STEP_OBJ_UP         5
#define STEP_BACKWARD       6
#define STEP_DONE           7
static int grab_step = STEP_WAIT;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cmd_pub;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mani_pub;

float object_x = 0.0;
float object_y = 0.0;
float object_z = 0.0;

float align_x = 1.0;
float align_y = 0.0;

void ObjectCallback(const wpr_simulation2::msg::Object::SharedPtr msg)
{
    if(grab_step == STEP_WAIT)
    {
        grab_step = STEP_ALIGN_OBJ;
    }
    if(grab_step == STEP_ALIGN_OBJ)
    {
        object_x = msg->x[0];
        object_y = msg->y[0];
        object_z = msg->z[0];
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("grab_object_node");

    cmd_pub = node->create_publisher<std_msgs::msg::String>(
        "/wpb_home/behavior", 
        10
    );
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 
        10
    );
    mani_pub = node->create_publisher<sensor_msgs::msg::JointState>(
        "/wpb_home/mani_ctrl",
         10
    );
    auto object_sub = node->create_subscription<wpr_simulation2::msg::Object>(
        "/wpb_home/objects_3d", 
        10, 
        ObjectCallback
    );
    
    rclcpp::Rate loop_rate(30);

    while(rclcpp::ok())
    {
        if(grab_step == STEP_WAIT)
        {
            std_msgs::msg::String start_msg;
            start_msg.data = "start objects";
            cmd_pub->publish(start_msg);
        }
        if(grab_step == STEP_ALIGN_OBJ)
        {
            float diff_x = object_x - align_x;
            float diff_y = object_y - align_y;
            geometry_msgs::msg::Twist vel_msg;
            if(fabs(diff_x) > 0.02 || fabs(diff_y) > 0.01)
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
                cmd_pub->publish(start_msg);
                grab_step = STEP_HAND_UP;
            }
            RCLCPP_INFO(
                node->get_logger(), 
                "[STEP_ALIGN_OBJ] vel = ( %.2f , %.2f )",
                vel_msg.linear.x,vel_msg.linear.y
            );
            vel_pub->publish(vel_msg);
        }
        if(grab_step == STEP_HAND_UP)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_HAND_UP]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z;
            mani_msg.position[1] = 0.15;
            mani_pub->publish(mani_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(8000));
            grab_step = STEP_FORWARD;
        }
        if(grab_step == STEP_FORWARD)
        {
            RCLCPP_INFO(
                node->get_logger(), 
                "[STEP_FORWARD] object_x = %.2f", 
                object_x
            );
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0.1;
            vel_msg.linear.y = 0.0;
            vel_pub->publish(vel_msg);
            int forward_duration = (object_x - 0.65) * 20000;
            rclcpp::sleep_for(std::chrono::milliseconds(forward_duration));
            grab_step = STEP_GRAB;
        }
        if(grab_step == STEP_GRAB)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_GRAB]");
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_pub->publish(vel_msg);
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z;
            mani_msg.position[1] = 0.07;
            mani_pub->publish(mani_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            grab_step = STEP_OBJ_UP;
        }
        if(grab_step == STEP_OBJ_UP)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_OBJ_UP]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z + 0.05;
            mani_msg.position[1] = 0.07;
            mani_pub->publish(mani_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(5000));
            grab_step = STEP_BACKWARD;
        }
        if(grab_step == STEP_BACKWARD)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_BACKWARD]");
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = -0.1;
            vel_msg.linear.y = 0;
            vel_pub->publish(vel_msg);
            rclcpp::sleep_for(std::chrono::milliseconds(10000));
            grab_step = STEP_DONE;
            RCLCPP_INFO(node->get_logger(), "[STEP_DONE]");
        }
        if(grab_step == STEP_DONE)
        {
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_pub->publish(vel_msg);
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}