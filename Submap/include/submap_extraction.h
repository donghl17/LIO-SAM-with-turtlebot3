#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include<tf/transform_listener.h>
#include <ros/duration.h>
#include<thread>
#include<mutex>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>	
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>

#include<Eigen/Dense>
#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <tf/transform_broadcaster.h>

#include "Submap/KMeans.h"
#include "Submap/GMM.h"

class Submap {
public:
  Submap() {  }
  ~Submap() {}
void initMap(ros::NodeHandle nh);
void pubSubinfo();
void Mapmerge();
void GMM_training(int novel_submap_frame);
void mapCallback(const sensor_msgs::PointCloud2 img);
Eigen::Matrix4f  TransformToMatrix(const tf::StampedTransform& transform) ;
void Global_Pointcloud_Publisher();
void Global_GMM_Publisher();
void Submap_GMM_building();

private:
    // ros::NodeHandle nh_;
    std::vector<sensor_msgs::PointCloud2> Submap_list_;
    std::vector<tf::StampedTransform> SubTF_list_;
    std::vector<GMM*> SubGMM_list_; // all the gmm submap are in the global TF, not local
    sensor_msgs::PointCloud2 Globalmap_; // to be changed into GMM formation
    pcl::PointCloud<pcl::PointXYZ> global_cloud_;
    int mapcnt_;// extract a Submap every mapcnt frames, to be improved as feature-based extractor
    ros::NodeHandle node_;
    ros::Publisher gobalmap_pub_; 
    // ros::Publisher submap_pub_; 
    // ros::Publisher subTF_pub_;
    ros::Subscriber map_sub_;
    tf::TransformListener TFlistener_;
    std::mutex Globalmap_mutex_;
    

};