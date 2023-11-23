#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/msg/twist.hpp>

std::shared_ptr<rclcpp::Node> node;
cv::Mat imgFace;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub;

int target_x = 960/2;

geometry_msgs::msg::Twist vel_cmd;   // Velocity message
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub; // Velocity publisher

void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    imgFace = cv_ptr->image;
    
    frame_pub->publish(*msg);
}

void FacePosCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
{
    int face_pose_x = msg->x_offset + (msg->width * 0.5);
    float fVelTurn = (target_x - face_pose_x) * 0.003;    // Difference * Proportion
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = fVelTurn;
    vel_pub->publish(vel_cmd);

    cv::rectangle(imgFace, 
        cv::Point(msg->x_offset, msg->y_offset), 
        cv::Point(msg->x_offset + msg->width, msg->y_offset + msg->height), 
        cv::Scalar(0, 0, 255), 
        2, 
        cv::LINE_8);
    cv::imshow("Face", imgFace);
    cv::waitKey(1);
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("cv_face_detect");

    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/kinect2/qhd/image_raw", 1, CamRGBCallback);
    frame_pub = node->create_publisher<sensor_msgs::msg::Image>(
        "/face_detector_input", 1);
    auto face_sub = node->create_subscription<sensor_msgs::msg::RegionOfInterest>(
        "/face_position", 1, FacePosCallback);
    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    cv::namedWindow("Face");

    rclcpp::spin(node);
    
    cv::destroyAllWindows();
    
    rclcpp::shutdown();

    return 0;
}