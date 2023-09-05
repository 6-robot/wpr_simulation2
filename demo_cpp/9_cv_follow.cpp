#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/msg/twist.hpp>

std::shared_ptr<rclcpp::Node> node;

using namespace cv;
using namespace std;

static int iLowH = 10;
static int iHighH = 40;

static int iLowS = 90; 
static int iHighS = 255;

static int iLowV = 1;
static int iHighV = 255;

geometry_msgs::msg::Twist vel_cmd;   // Velocity message
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub; // Velocity publisher

void Cam_RGB_Callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Mat imgOriginal = cv_ptr->image;

    // Convert the RGB image to HSV
    Mat imgHSV;
    vector<Mat> hsvSplit;
    cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

    // Perform histogram equalization in the HSV space
    split(imgHSV, hsvSplit);
    equalizeHist(hsvSplit[2],hsvSplit[2]);
    merge(hsvSplit,imgHSV);
    Mat imgThresholded;

    // Use the above Hue, Saturation, and Value threshold ranges to binarize the image
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 

    // Perform opening operation (remove some noise)
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

    // Perform closing operation (connect some connected domains)
    morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

    // Iterate through the binary image data
    int nTargetX = 0;
    int nTargetY = 0;
    int nPixCount = 0;
    int nImgWidth = imgThresholded.cols;
    int nImgHeight = imgThresholded.rows;
    int nImgChannels = imgThresholded.channels();
    printf("Width = %d   Height= %d \n",nImgWidth,nImgHeight);
    for (int y = 0; y < nImgHeight; y++)
    {
        for(int x = 0; x < nImgWidth; x++)
        {
            if(imgThresholded.data[y*nImgWidth + x] == 255)
            {
                nTargetX += x;
                nTargetY += y;
                nPixCount ++;
            }
        }
    }
    if(nPixCount > 0)
    {
        nTargetX /= nPixCount;
        nTargetY /= nPixCount;
        printf("Color centroid coordinates ( %d , %d )  Number of points = %d\n",nTargetX,nTargetY,nPixCount);
        // Draw coordinates
        Point line_begin = Point(nTargetX-10,nTargetY);
        Point line_end = Point(nTargetX+10,nTargetY);
        line(imgOriginal,line_begin,line_end,Scalar(255,0,0),3);
        line_begin.x = nTargetX; line_begin.y = nTargetY-10; 
        line_end.x = nTargetX; line_end.y = nTargetY+10; 
        line(imgOriginal,line_begin,line_end,Scalar(255,0,0),3);
        // Calculate robot motion speed
        float fVelFoward = (nImgHeight/2-nTargetY)*0.002;     // Difference * Proportion
        float fVelTurn = (nImgWidth/2-nTargetX)*0.003;           // Difference * Proportion
        vel_cmd.linear.x = fVelFoward;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = fVelTurn;
    }
    else
    {
        printf("Target color disappeared...\n");
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = 0;
    }

    // Publish the velocity command
    vel_pub->publish(vel_cmd);

    // Show the image
    imshow("Threshold", imgThresholded);
    imshow("Original", imgOriginal);
    cv::waitKey(5);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    node = std::make_shared<rclcpp::Node>("cv_follow_node");

    vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto sub = node->create_subscription<sensor_msgs::msg::Image>("/kinect2/qhd/image_color", 10, Cam_RGB_Callback);

    rclcpp::spin(node);
    cv::destroyAllWindows();
    rclcpp::shutdown();

    return 0;
}