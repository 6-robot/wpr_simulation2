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

class PointcloudObjects : public rclcpp::Node
{
public:
  PointcloudObjects()
  : Node("pointcloud_cluster")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/kinect2/sd/points", 10, std::bind(&PointcloudObjects::pointcloudCallback, this, std::placeholders::_1));
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr input)
  {
    // 将点云数值从相机坐标系转换到机器人坐标系
    bool result = tf_buffer_->canTransform("base_footprint", input->header.frame_id, input->header.stamp);
    if (!result)
    {
      return;
    }
    sensor_msgs::msg::PointCloud2 pc_footprint;
    pcl_ros::transformPointCloud("base_footprint", *input, pc_footprint, *tf_buffer_);

    // 将点云数据从ROS格式转换到PCL格式
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint, cloud_src);

    // 对设定范围内的点云进行截取
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    // 截取x轴方向，前方0.5米到1.5米内的点云
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("x");
    pass.setFilterLimits(0.5, 1.5);
    pass.filter(cloud_src);
    // 截取y轴方向，左侧0.5米到右侧0.5米内的点云
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.5, 0.5);
    pass.filter(cloud_src);
    // 截取z轴方向，高度0.5米到1.5米内的点云
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.5, 1.5);
    pass.filter(cloud_src);

    // 定义模型分类器
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    segmentation.setInputCloud(cloud_src.makeShared());
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.05);
    segmentation.setOptimizeCoefficients(true);

    // 使用模型分类器进行检测
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);

    // 统计平面点集的平均高度
    int point_num = planeIndices->indices.size();
    float points_z_sum = 0;
    for (int i = 0; i < point_num; i++)
    {
      int point_index = planeIndices->indices[i];
      points_z_sum += cloud_src.points[point_index].z;
    }
    float plane_height = points_z_sum / point_num;
    RCLCPP_INFO(this->get_logger(), "plane_height = %.2f", plane_height);

    // 对点云再次进行截取，只保留平面以上的部分
    pass.setInputCloud(cloud_src.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(plane_height + 0.2, 1.5);
    pass.filter(cloud_src);

    // 对截取后的点云进行欧式距离分割
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud_src.makeShared()); // 输入截取后的点云数据到KD树

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.1); // 设置聚类的容差
    ec.setMinClusterSize(100); // 设置每个聚类的最小点数
    ec.setMaxClusterSize(25000); // 设置每个聚类的最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_src.makeShared());
    ec.extract(cluster_indices);

    // 输出聚类的数量
    RCLCPP_INFO(this->get_logger(), "Number of clusters: %d", cluster_indices.size());

    // 计算每个分割出来的点云团的中心坐标
    int object_num = cluster_indices.size();                                       // 分割出的点云团个数
    RCLCPP_INFO(this->get_logger(), "object_num = %d",object_num);
    for(int i = 0 ; i < object_num ; i ++)
    {
        int point_num =  cluster_indices[i].indices.size();                 // 点云团i中的点数
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
        RCLCPP_INFO(this->get_logger(), "object %d pos = ( %.2f , %.2f , %.2f)" ,i, object_x,object_y,object_z);
    }
    RCLCPP_INFO(this->get_logger(), "---------------------" );
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;

  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointcloudObjects>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}