#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

std::shared_ptr<rclcpp::Node> node;

using namespace cv;
using namespace std;

static int iLowH = 10;
static int iHighH = 40;

static int iLowS = 90; 
static int iHighS = 255;

static int iLowV = 1;
static int iHighV = 255;

void CamRGBCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    Mat imgOriginal = cv_ptr->image;
    
    // Convert RGB image to HSV
    Mat imgHSV;
    vector<Mat> hsvSplit;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

    // Perform histogram equalization in the HSV space
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2], hsvSplit[2]);
    merge(hsvSplit, imgHSV);
    Mat imgThresholded;

    // Threshold the image using the specified Hue, Saturation, and Value thresholds
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 

    // Opening operation (remove some noise)
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

    // Closing operation (connect some regions)
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    // Traverse the binary image
    int nTargetX = 0;
    int nTargetY = 0;
    int nPixCount = 0;
    int nImgWidth = imgThresholded.cols;
    int nImgHeight = imgThresholded.rows;
    int nImgChannels = imgThresholded.channels();
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
        printf("Color centroid coordinates (%d, %d) Point count = %d\n", nTargetX, nTargetY, nPixCount);
        // Draw coordinates
        Point line_begin = Point(nTargetX - 10, nTargetY);
        Point line_end = Point(nTargetX + 10, nTargetY);
        line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0));
        line_begin.x = nTargetX; line_begin.y = nTargetY - 10; 
        line_end.x = nTargetX; line_end.y = nTargetY + 10; 
        line(imgOriginal, line_begin, line_end, Scalar(255, 0, 0));
    }
    else
    {
        printf("Target color disappeared...\n");
    }

    // Display the processed images
    imshow("RGB", imgOriginal);
    imshow("HSV", imgHSV);
    imshow("Result", imgThresholded);
    cv::waitKey(5);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("cv_hsv_node");

    auto rgb_sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/kinect2/qhd/image_raw", 1, CamRGBCallback);

    // Create windows for image display and parameter adjustment
    namedWindow("Threshold", WINDOW_AUTOSIZE);

    // createTrackbar( "LowH", "Threshold", nullptr, 179, 0 );
    // setTrackbarPos( "LowH", "Threshold", iLowH);
    createTrackbar("LowH", "Threshold", &iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Threshold", &iHighH, 179);

    createTrackbar("LowS", "Threshold", &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Threshold", &iHighS, 255);

    createTrackbar("LowV", "Threshold", &iLowV, 255); //Value (0 - 255)
    createTrackbar("HighV", "Threshold", &iHighV, 255);

    namedWindow("RGB"); 
    namedWindow("HSV"); 
    namedWindow("Result"); 

    rclcpp::spin(node);

    cv::destroyAllWindows();

    rclcpp::shutdown();
    return 0;
}