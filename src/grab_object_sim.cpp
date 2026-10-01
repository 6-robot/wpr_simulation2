#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wpr_simulation2/msg/object.hpp>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <limits>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#define STEP_WAIT           0
#define STEP_FIND_OBJ       1
#define STEP_ALIGN_OBJ      2
#define STEP_HAND_UP        3
#define STEP_FORWARD        4
#define STEP_GRAB           5
#define STEP_OBJ_UP         6
#define STEP_BACKWARD       7
#define STEP_DONE           8
static int grab_step = STEP_WAIT;

std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr mani_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr behavior_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub;

float object_x = 0.0;
float object_y = 0.0;
float object_z = 0.0;
std::shared_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
geometry_msgs::msg::Point target_odom;
bool target_locked = false;
bool odom_received = false;
double base_x = 0, base_y = 0, base_yaw = 0;
rclcpp::Time last_detection(0, 0, RCL_ROS_TIME);
rclcpp::Time last_odom(0, 0, RCL_ROS_TIME);
rclcpp::Time stable_since(0, 0, RCL_ROS_TIME);
rclcpp::Time alignment_started(0, 0, RCL_ROS_TIME);

void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    base_x = msg->pose.pose.position.x;
    base_y = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    base_yaw = std::atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z));
    last_odom = msg->header.stamp;
    odom_received = true;
}

void TargetInBase(double & x, double & y)
{
    const double dx = target_odom.x - base_x;
    const double dy = target_odom.y - base_y;
    x = std::cos(base_yaw)*dx + std::sin(base_yaw)*dy;
    y = -std::sin(base_yaw)*dx + std::cos(base_yaw)*dy;
}

float align_x = 1.0;
float align_y = 0.0;

// Keep refreshing the command while waiting for an action to finish.
// A single stop followed by a long sleep can leave the base using an old command.
bool HoldVelocity(const geometry_msgs::msg::Twist & velocity,
                  std::chrono::milliseconds duration)
{
    const auto deadline = node->now() + rclcpp::Duration::from_seconds(duration.count()/1000.0);
    rclcpp::WallRate rate(30);
    while(rclcpp::ok())
    {
        vel_pub->publish(velocity);
        rclcpp::spin_some(node);
        if(node->now() >= deadline)
        {
            return true;
        }
        rate.sleep();
    }
    return false;
}

void BehaviorCallback(const std_msgs::msg::String::SharedPtr msg)
{
    if(grab_step == STEP_WAIT && msg->data == "start grab")
    {
        std_msgs::msg::String msg;
        msg.data = "start objects";
        behavior_pub->publish(msg);
        target_locked = false;
        stable_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
        alignment_started = node->now();
        grab_step = STEP_FIND_OBJ;
    }
}

void ObjectCallback(const wpr_simulation2::msg::Object::SharedPtr msg)
{
    if(grab_step != STEP_FIND_OBJ && grab_step != STEP_ALIGN_OBJ)
        return;
    const double age = (node->now() - rclcpp::Time(msg->header.stamp)).seconds();
    if(msg->header.frame_id.empty() || age < -0.05 || age > 0.5)
        return;
    geometry_msgs::msg::TransformStamped transform;
    try
    {
        // Resolve the camera observation at capture time, before the base moved.
        transform = tf_buffer->lookupTransform("odom", msg->header.frame_id, msg->header.stamp);
    }
    catch(const tf2::TransformException &)
    {
        return;
    }
    const size_t size = std::min({msg->x.size(), msg->y.size(), msg->z.size()});
    double best_distance = target_locked ? 0.12 : std::numeric_limits<double>::infinity();
    bool found = false;
    geometry_msgs::msg::Point best;
    size_t best_index = 0;
    for(size_t i = 0; i < size; ++i)
    {
        if(!std::isfinite(msg->x[i]) || !std::isfinite(msg->y[i]) || !std::isfinite(msg->z[i]))
            continue;
        geometry_msgs::msg::PointStamped point, transformed;
        point.header = msg->header;
        point.point.x = msg->x[i]; point.point.y = msg->y[i]; point.point.z = msg->z[i];
        tf2::doTransform(point, transformed, transform);
        const auto & candidate = transformed.point;
        const double distance = target_locked ?
            std::hypot(std::hypot(candidate.x-target_odom.x, candidate.y-target_odom.y),
                       candidate.z-target_odom.z) : std::hypot(msg->x[i], msg->y[i]);
        if(distance < best_distance)
        {
            best_distance = distance;
            best = candidate;
            best_index = i;
            found = true;
        }
    }
    if(!found)
        return;
    target_odom = best;
    object_z = msg->z[best_index];
    last_detection = msg->header.stamp;
    if(!target_locked)
        RCLCPP_INFO(node->get_logger(), "[STEP_ALIGN_OBJ] target locked in odom (%.3f, %.3f)", best.x, best.y);
    target_locked = true;
    grab_step = STEP_ALIGN_OBJ;
}

void AbortGrab(const char * reason)
{
    RCLCPP_ERROR(node->get_logger(), "Grab aborted - %s", reason);
    HoldVelocity(geometry_msgs::msg::Twist{}, std::chrono::milliseconds(300));
    std_msgs::msg::String stop;
    stop.data = "stop objects";
    behavior_pub->publish(stop);
    stop.data = "grab failed";
    result_pub->publish(stop);
    grab_step = STEP_WAIT;
}

// Close the position loop with odometry instead of assuming wall time equals travel.
bool DriveTo(double goal_x, double goal_y)
{
    const auto deadline = node->now() + rclcpp::Duration::from_seconds(20.0);
    rclcpp::WallRate rate(30);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if(!odom_received || (node->now()-last_odom).seconds() > 0.5 || node->now() > deadline)
        {
            AbortGrab("odometry stale or approach timed out");
            return false;
        }
        const double dx = goal_x-base_x, dy = goal_y-base_y;
        if(std::hypot(dx, dy) < 0.005)
            return HoldVelocity(geometry_msgs::msg::Twist{}, std::chrono::milliseconds(300));
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(0.8*(std::cos(base_yaw)*dx+std::sin(base_yaw)*dy), -0.1, 0.1);
        cmd.linear.y = std::clamp(0.8*(-std::sin(base_yaw)*dx+std::cos(base_yaw)*dy), -0.05, 0.05);
        vel_pub->publish(cmd);
        rate.sleep();
    }
    return false;
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("grab_node",
        rclcpp::NodeOptions().append_parameter_override("use_sim_time", true));
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, OdomCallback);

    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    mani_pub = node->create_publisher<sensor_msgs::msg::JointState>("/wpb_home/mani_ctrl", 10);
    auto object_sub = node->create_subscription<wpr_simulation2::msg::Object>("/wpb_home/objects_3d", 1, ObjectCallback);
    behavior_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/behavior", 10);
    auto behavior_sub = node->create_subscription<std_msgs::msg::String>("/wpb_home/behavior", 10, BehaviorCallback);
    result_pub = node->create_publisher<std_msgs::msg::String>("/wpb_home/grab_result", 10);
    
    rclcpp::WallRate loop_rate(30);

    while(rclcpp::ok())
    {
        rclcpp::spin_some(node);
        loop_rate.sleep();
        if(grab_step == STEP_FIND_OBJ || grab_step == STEP_ALIGN_OBJ)
        {
            geometry_msgs::msg::Twist vel_msg;
            if((node->now()-alignment_started).seconds() > 30.0)
            {
                AbortGrab("no stable target within 30 seconds");
                continue;
            }
            if(!target_locked || !odom_received ||
               (node->now()-last_detection).seconds() > 0.5 ||
               (node->now()-last_odom).seconds() > 0.5)
            {
                stable_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
                vel_pub->publish(vel_msg);
                continue;
            }
            double x, y;
            TargetInBase(x, y);
            object_x = x; object_y = y;
            const double diff_x = x-align_x, diff_y = y-align_y;
            if(std::abs(diff_x) > 0.01 || std::abs(diff_y) > 0.003)
            {
                stable_since = rclcpp::Time(0, 0, RCL_ROS_TIME);
                vel_msg.linear.x = std::clamp(diff_x*0.6, -0.12, 0.12);
                vel_msg.linear.y = std::clamp(diff_y*0.6, -0.12, 0.12);
            }
            else
            {
                if(stable_since.nanoseconds() == 0)
                    stable_since = node->now();
                if((node->now()-stable_since).seconds() >= 0.3)
                {
                    grab_step = STEP_HAND_UP;
                    std_msgs::msg::String msg;
                    msg.data = "stop objects";
                    behavior_pub->publish(msg);
                    RCLCPP_INFO(node->get_logger(), "Alignment settled at (%.3f, %.3f, %.3f)", x, y, object_z);
                }
            }
            RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 500,
                "[STEP_ALIGN_OBJ] vel = ( %.3f , %.3f )", vel_msg.linear.x,vel_msg.linear.y);
            vel_pub->publish(vel_msg);
            continue;
        }
        if(grab_step == STEP_HAND_UP)
        {
            // Send several stop commands before starting the lift.
            if(!HoldVelocity(geometry_msgs::msg::Twist{}, std::chrono::milliseconds(200)))
                break;
            RCLCPP_INFO(node->get_logger(), "[STEP_HAND_UP]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z + 0.04;
            mani_msg.position[1] = 0.15;
            mani_pub->publish(mani_msg);
            if(!HoldVelocity(geometry_msgs::msg::Twist{}, std::chrono::milliseconds(8000)))
                break;
            grab_step = STEP_FORWARD;
            continue;
        }
        if(grab_step == STEP_FORWARD)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_FORWARD] object_x = %.2f", object_x);
            const double goal_x = target_odom.x - 0.65*std::cos(base_yaw);
            const double goal_y = target_odom.y - 0.65*std::sin(base_yaw);
            if(!DriveTo(goal_x, goal_y))
                continue;
            grab_step = STEP_GRAB;
            continue;
        }
        if(grab_step == STEP_GRAB)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_GRAB]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z + 0.04;
            mani_msg.position[1] = 0.058;
            mani_pub->publish(mani_msg);
            geometry_msgs::msg::Twist vel_msg;
            vel_msg.linear.x = 0;
            vel_msg.linear.y = 0;
            vel_pub->publish(vel_msg);
            if(!HoldVelocity(geometry_msgs::msg::Twist{}, std::chrono::milliseconds(5000)))
                break;
            grab_step = STEP_OBJ_UP;
            continue;
        }
        if(grab_step == STEP_OBJ_UP)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_OBJ_UP]");
            sensor_msgs::msg::JointState mani_msg;
            mani_msg.name.resize(2);
            mani_msg.name[0] = "lift";
            mani_msg.name[1] = "gripper";
            mani_msg.position.resize(2);
            mani_msg.position[0] = object_z + 0.09;
            mani_msg.position[1] = 0.058;
            mani_pub->publish(mani_msg);
            if(!HoldVelocity(geometry_msgs::msg::Twist{}, std::chrono::milliseconds(5000)))
                break;
            grab_step = STEP_BACKWARD;
            continue;
        }
        if(grab_step == STEP_BACKWARD)
        {
            RCLCPP_INFO(node->get_logger(), "[STEP_BACKWARD]");
            if(!DriveTo(base_x-0.5*std::cos(base_yaw), base_y-0.5*std::sin(base_yaw)))
                continue;
            // Refresh stop before reporting completion to the next behavior.
            if(!HoldVelocity(geometry_msgs::msg::Twist{}, std::chrono::milliseconds(1000)))
                break;
            grab_step = STEP_DONE;
            RCLCPP_INFO(node->get_logger(), "[STEP_DONE]");
            continue;
        }
        if(grab_step == STEP_DONE)
        {
            // The base has stopped; hand control back before navigation starts.
            std_msgs::msg::String res_msg;
            res_msg.data = "grab done";
            result_pub->publish(res_msg);
            grab_step = STEP_WAIT;
        }
    }

    rclcpp::shutdown();

    return 0;
}
