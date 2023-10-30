#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

std::shared_ptr<rclcpp::Node> node;

void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ> pointCloudIn;
  pcl::fromROSMsg(*msg, pointCloudIn);

  int cloudSize = pointCloudIn.points.size();
  for (int i = 0; i < cloudSize; i++)
  {
    RCLCPP_INFO(node->get_logger(), "[i= %d] ( %.2f , %.2f , %.2f)",
                i,
                pointCloudIn.points[i].x,
                pointCloudIn.points[i].y,
                pointCloudIn.points[i].z);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("pointcloud_data_node");
  auto pc_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/kinect2/sd/points",
    1,
    PointcloudCallback
  );

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}