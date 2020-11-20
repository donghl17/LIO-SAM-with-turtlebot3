#include "submap_extraction.h"

//once in a time


int main(int argc, char** argv)
{
  ros::init(argc, argv, "globalmap_publisher");
//   ros::NodeHandle node;
  ros::NodeHandle nh("~");
  std::cout<<"Robot1 begin"<<std::endl;
  Submap Robot1;
  Robot1.initMap(nh);
  std::cout<<"Robot1 end"<<std::endl;
    ros::spin();
  return 0;
}