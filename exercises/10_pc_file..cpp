#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

std::shared_ptr<rclcpp::Node> node;
bool save_flag = true;

void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZ> pointCloudIn;
  pcl::fromROSMsg(*msg, pointCloudIn);

  if(save_flag == true)
  {
    pcl::io::savePCDFileASCII("pointcloud_data.pcd", pointCloudIn);
    RCLCPP_INFO(node->get_logger(), "Point cloud data saved as pointcloud_data.pcd");
    save_flag = false;
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