#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

std::shared_ptr<rclcpp::Node> node;

void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion tf2_quaternion;
    tf2_quaternion.setX(msg->orientation.x);
    tf2_quaternion.setY(msg->orientation.y);
    tf2_quaternion.setZ(msg->orientation.z);
    tf2_quaternion.setW(msg->orientation.w);

    tf2::Matrix3x3 matrix(tf2_quaternion);

    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;
    RCLCPP_INFO(node->get_logger(), "roll= %.0f pitch= %.0f yaw= %.0f", roll, pitch, yaw);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("imu_data_node");

    auto sub = node->create_subscription<sensor_msgs::msg::Imu>("/imu/data", 10, IMUCallback);

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}