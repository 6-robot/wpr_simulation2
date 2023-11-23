#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudNode : public rclcpp::Node
{
public:
  PointCloudNode()
    : Node("pointcloud_data_node")
  {
    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/kinect2/sd/points",
      1,
      std::bind(&PointCloudNode::PointcloudCallback, this, std::placeholders::_1));

    // Initialize point cloud data
    pointCloudData_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  }

private:
  void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::fromROSMsg(*msg, *pointCloudData_);

    int cloudSize = pointCloudData_->points.size();
    for (int i = 0; i < cloudSize; i++)
    {
      RCLCPP_INFO(get_logger(), "[i= %d] ( %.2f , %.2f , %.2f)",
                  i,
                  pointCloudData_->points[i].x,
                  pointCloudData_->points[i].y,
                  pointCloudData_->points[i].z);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudData_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PointCloudNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}