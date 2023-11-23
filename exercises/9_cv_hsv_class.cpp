#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class CVHSVNode : public rclcpp::Node
{
public:
  CVHSVNode() : Node("cv_hsv_node")
  {
    rgb_subscriber_ = create_subscription<sensor_msgs::msg::Image>(
      "/kinect2/qhd/image_raw", 1, std::bind(&CVHSVNode::CamRGBCallback, this, std::placeholders::_1)
      );

    cv::namedWindow("Threshold", cv::WINDOW_AUTOSIZE);

    cv::createTrackbar("LowH", "Threshold", &iLowH_, 179); //Hue (0 - 179)
    cv::createTrackbar("HighH", "Threshold", &iHighH_, 179);

    cv::createTrackbar("LowS", "Threshold", &iLowS_, 255); //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Threshold", &iHighS_, 255);

    cv::createTrackbar("LowV", "Threshold", &iLowV_, 255); //Value (0 - 255)
    cv::createTrackbar("HighV", "Threshold", &iHighV_, 255);

    iLowH_ = 10;
    iHighH_ = 40;
    iLowS_ = 90; 
    iHighS_ = 255;
    iLowV_ = 1;
    iHighV_ = 255;

    cv::namedWindow("RGB");
    cv::namedWindow("HSV");
    cv::namedWindow("Result");
  }

private:
  void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
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
      cv::line(imgOriginal, line_begin, line_end, cv::Scalar(255, 0, 0));
      line_begin.x = nTargetX;
      line_begin.y = nTargetY - 10;
      line_end.x = nTargetX;
      line_end.y = nTargetY + 10;
      cv::line(imgOriginal, line_begin, line_end, cv::Scalar(255, 0, 0));
    }
    else
    {
      printf("Target disappeared...\n");
    }

    // Display the processed images
    imshow("RGB", imgOriginal);
    imshow("HSV", imgHSV);
    imshow("Result", imgThresholded);
    cv::waitKey(5);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_subscriber_;
  int iLowH_;
  int iHighH_;
  int iLowS_;
  int iHighS_;
  int iLowV_;
  int iHighV_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CVHSVNode>();
  rclcpp::spin(node);
  cv::destroyAllWindows();
  rclcpp::shutdown();
  return 0;
}