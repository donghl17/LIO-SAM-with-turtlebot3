#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include<tf/transform_listener.h>
#include <ros/duration.h>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>	
#include <pcl/point_types.h>

#include<Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <tf/transform_broadcaster.h>
class Submap {
public:
  Submap() {}
  ~Submap() {}
void initMap(ros::NodeHandle nh);
void pubSubinfo();
void Mapmerge();
void mapCallback(const sensor_msgs::PointCloud2 img);
Eigen::Matrix4f  TransformToMatrix(const tf::StampedTransform& transform) ;

private:
    // ros::NodeHandle nh_;
    std::vector<sensor_msgs::PointCloud2> Submap_list_;
    std::vector<tf::StampedTransform> SubTF_list_;
    sensor_msgs::PointCloud2 Globalmap_; // to be changed into GMM formation
    int mapcnt_;// extract a Submap every mapcnt frames, to be improved as feature-based extractor
    ros::NodeHandle node_;
    ros::Publisher gobalmap_pub_; 
    // ros::Publisher submap_pub_; 
    ros::Subscriber map_sub_;
    tf::TransformListener TFlistener_;
    tf::TransformBroadcaster br_;


};