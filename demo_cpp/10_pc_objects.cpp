#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

std::shared_ptr<rclcpp::Node> node;
tf2_ros::Buffer::SharedPtr tf_buffer_;
std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

void PointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  bool result = tf_buffer_->canTransform(
    "base_footprint", 
    msg->header.frame_id, 
    msg->header.stamp
    );
  if (!result)
  {
    return;
  }
  sensor_msgs::msg::PointCloud2 pc_footprint;
  pcl_ros::transformPointCloud(
    "base_footprint", 
    *msg, 
    pc_footprint, 
    *tf_buffer_
    );

  pcl::PointCloud<pcl::PointXYZ> cloud_src;
  pcl::fromROSMsg(pc_footprint, cloud_src);

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_src.makeShared());
  pass.setFilterFieldName("x");
  pass.setFilterLimits(0.5, 1.5);
  pass.filter(cloud_src);
  pass.setInputCloud(cloud_src.makeShared());
  pass.setFilterFieldName("y");
  pass.setFilterLimits(-0.5, 0.5);
  pass.filter(cloud_src);
  pass.setInputCloud(cloud_src.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(0.5, 1.5);
  pass.filter(cloud_src);

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::SACSegmentation<pcl::PointXYZ> segmentation;
  segmentation.setInputCloud(cloud_src.makeShared());
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.05);
  segmentation.setOptimizeCoefficients(true);

  pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
  segmentation.segment(*planeIndices, *coefficients);

  int point_num = planeIndices->indices.size();
  float points_z_sum = 0;
  for (int i = 0; i < point_num; i++)
  {
    int point_index = planeIndices->indices[i];
    points_z_sum += cloud_src.points[point_index].z;
  }
  float plane_height = points_z_sum / point_num;
  RCLCPP_INFO(node->get_logger(), "plane_height = %.2f", plane_height);

  pass.setInputCloud(cloud_src.makeShared());
  pass.setFilterFieldName("z");
  pass.setFilterLimits(plane_height + 0.2, 1.5);
  pass.filter(cloud_src);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud_src.makeShared()); 

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.1);
  ec.setMinClusterSize(100);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_src.makeShared());
  ec.extract(cluster_indices);
  int object_num = cluster_indices.size(); 
  RCLCPP_INFO(node->get_logger(), "object_num = %d",object_num);
  for(int i = 0 ; i < object_num ; i ++)
  {
      int point_num =  cluster_indices[i].indices.size();
      float points_x_sum = 0;
      float points_y_sum = 0;
      float points_z_sum = 0;
      for(int j = 0 ; j < point_num ; j ++)
      {
          int point_index = cluster_indices[i].indices[j];
          points_x_sum += cloud_src.points[point_index].x;
          points_y_sum += cloud_src.points[point_index].y;
          points_z_sum += cloud_src.points[point_index].z;
      }
      float object_x = points_x_sum/point_num;
      float object_y = points_y_sum/point_num;
      float object_z = points_z_sum/point_num;
      RCLCPP_INFO(
          node->get_logger(), 
          "object-%d pos=( %.2f , %.2f , %.2f)" ,
          i, 
          object_x,
          object_y,
          object_z
        );
  }
  RCLCPP_INFO(node->get_logger(), "---------------------" );
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("pointcloud_objects_node");

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto pc_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/kinect2/sd/points",
    1,
    PointcloudCallback
  );

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}