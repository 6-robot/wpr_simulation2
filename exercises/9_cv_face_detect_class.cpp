#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class FaceDetectionNode : public rclcpp::Node
{
public:
    FaceDetectionNode()
        : Node("face_detection_node")
    {
        // Create subscriptions
        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/kinect2/qhd/image_raw", 1, std::bind(&FaceDetectionNode::CamRGBCallback, this, std::placeholders::_1)
            );

        face_sub_ = create_subscription<sensor_msgs::msg::RegionOfInterest>(
            "/face_position", 1, std::bind(&FaceDetectionNode::FacePosCallback, this, std::placeholders::_1)
            );

        // Create publisher
        frame_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/face_detector_input", 1
            );

        // Create window for face display
        cv::namedWindow("Face");
    }

private:
    void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        imgFace_ = cv_ptr->image;

        frame_pub_->publish(*msg);
    }

    void FacePosCallback(const sensor_msgs::msg::RegionOfInterest::SharedPtr msg)
    {
        cv::rectangle(imgFace_,
            cv::Point(msg->x_offset, msg->y_offset),
            cv::Point(msg->x_offset + msg->width, msg->y_offset + msg->height),
            cv::Scalar(0, 0, 255),
            2,
            cv::LINE_8);

        cv::imshow("Face", imgFace_);
        cv::waitKey(1);
    }

    cv::Mat imgFace_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::RegionOfInterest>::SharedPtr face_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr frame_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaceDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}