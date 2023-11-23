#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 
#include <geometry_msgs/msg/twist.hpp>

class CVFollowerNode : public rclcpp::Node
{
public:
  CVFollowerNode() : Node("cv_follow_node")
  {
    vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
        "/kinect2/qhd/image_raw",
        10,
        std::bind(&CVFollowerNode::camRGBCallback, this, std::placeholders::_1)
        );

    iLowH_ = 10;
    iHighH_ = 40;
    iLowS_ = 90;
    iHighS_ = 255;
    iLowV_ = 1;
    iHighV_ = 255;

    cv::namedWindow("Result");
    cv::namedWindow("Original");
  }

private:
  void camRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::Mat imgOriginal = cv_ptr->image;

    // Convert RGB image to HSV
    cv::Mat imgHSV;
    cv::cvtColor(imgOriginal, imgHSV, cv::COLOR_BGR2HSV);

    // Perform histogram equalization in the HSV space
    std::vector<cv::Mat> hsvSplit;
    cv::split(imgHSV, hsvSplit);
    cv::equalizeHist(hsvSplit[2], hsvSplit[2]);
    cv::merge(hsvSplit, imgHSV);

    // Threshold the image using the specified Hue, Saturation, and Value thresholds
    cv::Mat imgThresholded;
    cv::inRange(imgHSV,
                cv::Scalar(iLowH_, iLowS_, iLowV_),
                cv::Scalar(iHighH_, iHighS_, iHighV_),
                imgThresholded
                );

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    // Opening operation (remove some noise)
    cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element);
    // Closing operation (connect some regions)
    cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, element);

    // Traverse the binary image
    int nTargetX = 0;
    int nTargetY = 0;
    int nPixCount = 0;
    int nImgWidth = imgThresholded.cols;
    int nImgHeight = imgThresholded.rows;
    for (int y = 0; y < nImgHeight; y++)
    {
      for (int x = 0; x < nImgWidth; x++)
      {
        if (imgThresholded.data[y * nImgWidth + x] == 255)
        {
          nTargetX += x;
          nTargetY += y;
          nPixCount++;
        }
      }
    }
    if (nPixCount > 0)
    {
      nTargetX /= nPixCount;
      nTargetY /= nPixCount;
      printf("Target (%d, %d) PixelCount = %d\n", nTargetX, nTargetY, nPixCount);
      // Draw coordinates
      cv::Point line_begin = cv::Point(nTargetX - 10, nTargetY);
      cv::Point line_end = cv::Point(nTargetX + 10, nTargetY);
      cv::line(imgOriginal, line_begin, line_end, cv::Scalar(255, 0, 0), 3);
      line_begin.x = nTargetX;
      line_begin.y = nTargetY - 10;
      line_end.x = nTargetX;
      line_end.y = nTargetY + 10;
      cv::line(imgOriginal, line_begin, line_end, cv::Scalar(255, 0, 0), 3);
      // Calculate robot motion speed
      float fVelFoward = (nImgHeight / 2 - nTargetY) * 0.002; // Difference * Proportion
      float fVelTurn = (nImgWidth / 2 - nTargetX) * 0.003;    // Difference * Proportion
      vel_cmd_.linear.x = fVelFoward;
      vel_cmd_.linear.y = 0;
      vel_cmd_.linear.z = 0;
      vel_cmd_.angular.x = 0;
      vel_cmd_.angular.y = 0;
      vel_cmd_.angular.z = fVelTurn;
    }
    else
    {
      printf("Target disappeared...\n");
      vel_cmd_.linear.x = 0;
      vel_cmd_.linear.y = 0;
      vel_cmd_.linear.z = 0;
      vel_cmd_.angular.x = 0;
      vel_cmd_.angular.y = 0;
      vel_cmd_.angular.z = 0;
    }

    // Publish the velocity command
    vel_pub_->publish(vel_cmd_);

    // Show the image
    cv::imshow("Result", imgThresholded);
    cv::imshow("Original", imgOriginal);
    cv::waitKey(5);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  int iLowH_;
  int iHighH_;
  int iLowS_;
  int iHighS_;
  int iLowV_;
  int iHighV_;

  geometry_msgs::msg::Twist vel_cmd_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CVFollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}