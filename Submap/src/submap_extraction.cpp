
#include "submap_extraction.h"
void Submap::initMap(ros::NodeHandle nh_){
    mapcnt_=0;
    std::cout<<"init begin"<<std::endl;
    //second param is queue length
    map_sub_=nh_.subscribe<sensor_msgs::PointCloud2>("/lio_sam/deskew/cloud_deskewed", 1000, &Submap::mapCallback,this);///camera/depth/points
    gobalmap_pub_=nh_.advertise<sensor_msgs::PointCloud2>("/globalmap",1000,this);

 
//   tf::Transform transform_temp;
//   transform_temp.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
//   tf::Quaternion q_temp;
//   q_temp.setRPY(0, 0, 0);
//   transform_temp.setRotation(q_temp);
//   br_.sendTransform(tf::StampedTransform(transform_temp, ros::Time::now(),  "odom","map"));

    std::cout<<"init end"<<std::endl;
}
 
 void Submap::Mapmerge(){
    // pubSubinfo();
    pcl::PointCloud<pcl::PointXYZ> cloud_1;
    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    pcl::PointCloud<pcl::PointXYZ> temp;
    for (int i=0; i<Submap_list_.size(); i++)
    {
        pcl::fromROSMsg(Submap_list_[i],cloud_1);
        Eigen::Matrix4f trans;
        // trans=TransformToMatrix(SubTF_list_[i]);
        Eigen::Matrix3f rotation_temp;
        Eigen::Quaternionf quaternion_temp(
            SubTF_list_[i].getRotation().getW(),SubTF_list_[i].getRotation().getX(),SubTF_list_[i].getRotation().getY(),SubTF_list_[i].getRotation().getZ());
        rotation_temp=quaternion_temp.toRotationMatrix();
        trans(0,0)=rotation_temp(0,0);
        trans(0,1)=rotation_temp(0,1);
        trans(0,2)=rotation_temp(0,2);
        trans(1,0)=rotation_temp(1,0);
        trans(1,1)=rotation_temp(1,1);
        trans(1,2)=rotation_temp(1,2);
        trans(2,0)=rotation_temp(2,0);
        trans(2,1)=rotation_temp(2,1);
        trans(2,2)=rotation_temp(2,2);
        trans(0,3)=SubTF_list_[i].getOrigin().getX();
        trans(1,3)=SubTF_list_[i].getOrigin().getY();
        trans(2,3)=SubTF_list_[i].getOrigin().getZ();
        std::cout<<trans<<std::endl;
        pcl::transformPointCloud(cloud_1, temp, trans);
        global_cloud = global_cloud + temp;
    }
    // pub GlobalMap
    pcl::toROSMsg(global_cloud,Globalmap_);// to dp: add other info, like tf to this msg
    Globalmap_.header.frame_id="/map";
            while (true){
                gobalmap_pub_.publish(Globalmap_);
                // ros::spinOnce();
                ros::Duration(1).sleep();
            }
    
    std::cout<<"publish Globalmap finish"<<std::endl;
}

Eigen::Matrix4f  Submap::TransformToMatrix(const tf::StampedTransform& transform) 
{
    Eigen::Matrix4f transform_matrix;
    Eigen::Translation3f tl_btol(
    transform.getOrigin().getX(), 
    transform.getOrigin().getY(), 
    transform.getOrigin().getZ());
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
    transform_matrix = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
    return transform_matrix;
}

//Question: Can we make sure that the img&pose are presenting the same frame of map in this way?
void Submap::mapCallback(sensor_msgs::PointCloud2 img){
    if (mapcnt_<20){
        mapcnt_++;
        std::cout<<mapcnt_<<std::endl;
    }
    else
    {
        std::cout<<"receive submap"<<std::endl;
        //to do: GMMmap_generate(img);
        tf::StampedTransform trans_temp;
        Submap_list_.push_back(img);
        TFlistener_.lookupTransform( "/map","/camera_depth_optical_frame",ros::Time(0), trans_temp);
        //output is the transform form "/camera_depth_optical_frame" to "/map"
        SubTF_list_.push_back(trans_temp);
        
        if (Submap_list_.size()==3)
        {
            std::cout<<"begin merging"<<std::endl;
            Mapmerge();
            // ros::shutdown();
        }
        mapcnt_=0;
    }
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "Submap_building");
//   ros::NodeHandle node;
  ros::NodeHandle nh("~");
  std::cout<<"Robot1 begin"<<std::endl;
  Submap Robot1;
  Robot1.initMap(nh);
  std::cout<<"Robot1 end"<<std::endl;
    ros::spin();
  return 0;
}