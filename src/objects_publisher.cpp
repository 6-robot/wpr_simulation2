#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <wpr_simulation2/msg/object.hpp>

using namespace std;

typedef struct stBoxMarker
{
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
}stBoxMarker;

typedef struct stObjectDetected
{
    string name;
    float x;
    float y;
    float z;
    float probability;
}stObjectDetected;

class ObjectsPublisher : public rclcpp::Node
{
public:
  ObjectsPublisher()
  : Node("ObjectsPublisher")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/kinect2/sd/points", 10, std::bind(&ObjectsPublisher::pointcloudCallback, this, std::placeholders::_1));
    objects_pub_ = this->create_publisher<wpr_simulation2::msg::Object>("/wpb_home/objects_3d", 10);
    marker_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/objects_marker", 10);
    nTextNum = 2;
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
    pass.setFilterLimits(plane_height + 0.02, 1.5);
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

    // 清除上一次的方框显示
    RemoveBoxes();
    arObj.clear();
    int nObjCnt = 0;

    // 计算每个分割出来的点云团的中心坐标
    int object_num = cluster_indices.size();                                       // 分割出的点云团个数
    RCLCPP_INFO(this->get_logger(), "object_num = %d",object_num);
    for(int i = 0 ; i < object_num ; i ++)
    {
        int point_num =  cluster_indices[i].indices.size();                 // 点云团i中的点数
        float points_x_sum = 0;
        float points_y_sum = 0;
        float points_z_sum = 0;
        bool bFirstPoint = true;
        for(int j = 0 ; j < point_num ; j ++)
        {
            int point_index = cluster_indices[i].indices[j];
            points_x_sum += cloud_src.points[point_index].x;
            points_y_sum += cloud_src.points[point_index].y;
            points_z_sum += cloud_src.points[point_index].z;
            // 物体的尺寸
            pcl::PointXYZRGB p = cloud_src.points[point_index];
            if(bFirstPoint == true)
            {
                boxMarker.xMax = boxMarker.xMin = p.x;
                boxMarker.yMax = boxMarker.yMin = p.y;
                boxMarker.zMax = boxMarker.zMin = p.z;
                bFirstPoint = false;
            }

            if(p.x < boxMarker.xMin) { boxMarker.xMin = p.x;}
            if(p.x > boxMarker.xMax) { boxMarker.xMax = p.x;}
            if(p.y < boxMarker.yMin) { boxMarker.yMin = p.y;}
            if(p.y > boxMarker.yMax) { boxMarker.yMax = p.y;}
            if(p.z < boxMarker.zMin) { boxMarker.zMin = p.z;}
            if(p.z > boxMarker.zMax) { boxMarker.zMax = p.z;}
        }
        float object_x = points_x_sum/point_num;
        float object_y = points_y_sum/point_num;
        float object_z = points_z_sum/point_num;
        RCLCPP_INFO(this->get_logger(), "object %d pos = ( %.2f , %.2f , %.2f)" ,i, object_x,object_y,object_z);
        
        if(boxMarker.xMin < 1.5 && boxMarker.yMin > -0.5 && boxMarker.yMax < 0.5)
        {
            DrawBox(boxMarker.xMin, boxMarker.xMax, boxMarker.yMin, boxMarker.yMax, boxMarker.zMin, boxMarker.zMax, 0, 1, 0, 2*i);

            std::ostringstream stringStream;
            stringStream << "object_" << nObjCnt;
            std::string obj_id = stringStream.str();
            float object_x = boxMarker.xMax;
            float object_y = (boxMarker.yMin+boxMarker.yMax)/2;
            float object_z = boxMarker.zMin;
            DrawText(obj_id,0.06, object_x,object_y,boxMarker.zMax+0.02, 1,0,1, 2*i+1);
            stObjectDetected tmpObj;
            tmpObj.name = obj_id;
            tmpObj.x = object_x;
            tmpObj.y = object_y;
            tmpObj.z = object_z;
            tmpObj.probability = 1.0f;
            arObj.push_back(tmpObj);

            // coord.name.push_back(obj_id);
            // coord.x.push_back(object_x);
            // coord.y.push_back(object_y);
            // coord.z.push_back(object_z);
            // coord.probability.push_back(1.0f);
            nObjCnt++;
            RCLCPP_INFO(this->get_logger(),"[obj_%d] xMin= %.2f yMin = %.2f yMax = %.2f",i,boxMarker.xMin, boxMarker.yMin, boxMarker.yMax);
        } 
    }
    SortObjects();
    marker_pub->publish(marker_array);
    RCLCPP_INFO(this->get_logger(), "---------------------" );
  }

  void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB, int inID)
  {
      line_box.header.frame_id = "base_footprint";
      line_box.header.stamp = rclcpp::Clock().now();
      line_box.ns = "line_box";
      line_box.action = visualization_msgs::msg::Marker::ADD;
      line_box.id = inID;
      line_box.type = visualization_msgs::msg::Marker::LINE_LIST;
      line_box.scale.x = 0.005;
      line_box.color.r = inR;
      line_box.color.g = inG;
      line_box.color.b = inB;
      line_box.color.a = 1.0;

      line_box.points.clear();
      geometry_msgs::msg::Point p;
      p.z = inMinZ;
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

      p.z = inMaxZ;
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

      p.x = inMinX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMinX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

      p.x = inMaxX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
      p.x = inMaxX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

      marker_array.markers.push_back(line_box);
  }

  void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB, int inID)
  {
      text_marker.header.frame_id = "base_footprint";
      text_marker.header.stamp = rclcpp::Clock().now();
      text_marker.id = inID;
      text_marker.ns = "line_obj";
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      text_marker.id = nTextNum;
      nTextNum++;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.scale.z = inScale;
      text_marker.color.r = inR;
      text_marker.color.g = inG;
      text_marker.color.b = inB;
      text_marker.color.a = 1.0;

      text_marker.pose.position.x = inX;
      text_marker.pose.position.y = inY;
      text_marker.pose.position.z = inZ;

      text_marker.text = inText;

      marker_array.markers.push_back(text_marker);
  }

  void RemoveBoxes()
  {
      marker_array.markers.clear();
  }

  float CalObjDist(stObjectDetected* inObj)
  {
      float x = inObj->x;
      float y = inObj->y;
      float z = inObj->z - 0.8f;
      float dist = sqrt(x*x + y*y + z*z);
      return dist;
  }

  void SortObjects()
  {
      int nNum = arObj.size();
      if (nNum == 0)
          return;
      // 冒泡排序
      stObjectDetected tObj;
      for(int n = 0; n<nNum; n++)
      {
          float minObjDist = CalObjDist(&arObj[n]);
          for(int i=n+1;i<nNum; i++)
          {
              float curDist = CalObjDist(&arObj[i]);
              if(curDist < minObjDist)
              {
                  // 交换位置
                  tObj = arObj[n];
                  arObj[n] = arObj[i];
                  arObj[i] = tObj;
                  minObjDist = curDist;
              }
          }
      }
      // 排序完毕，发送消息
      wpr_simulation2::msg::Object object_msg;
      for(int i=0;i<nNum; i++)
      {
          object_msg.name.push_back(arObj[i].name);
          object_msg.x.push_back(arObj[i].x);
          object_msg.y.push_back(arObj[i].y);
          object_msg.z.push_back(arObj[i].z);
          object_msg.probability.push_back(arObj[i].probability);
      }
      objects_pub_->publish(object_msg);
  }

  tf2_ros::Buffer::SharedPtr tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<wpr_simulation2::msg::Object>::SharedPtr objects_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
  stBoxMarker boxMarker;
  std::vector<stObjectDetected> arObj;
  visualization_msgs::msg::Marker line_box;
  visualization_msgs::msg::Marker line_follow;
  visualization_msgs::msg::Marker text_marker;

  visualization_msgs::msg::MarkerArray marker_array;
  int nTextNum;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjectsPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}