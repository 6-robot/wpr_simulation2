#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class CVImageNode : public rclcpp::Node
{
public:
  CVImageNode() : Node("cv_image_node")
  {
    rgb_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
      "/kinect2/qhd/image_raw", 1, std::bind(&CVImageNode::CamRGBCallback, this, std::placeholders::_1));

    cv::namedWindow("RGB");
  }

private:
  void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat imgOriginal = cv_ptr->image;
    cv::imshow("RGB", imgOriginal);
    cv::waitKey(1);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscriber_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CVImageNode>();
  rclcpp::spin(node);
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}