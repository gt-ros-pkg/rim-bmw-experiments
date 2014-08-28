#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/registration/transforms.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>

//#include<bmw_percep/pcl_cv_utils.hpp>
//#include<bmw_percep/groundPlane.hpp>

//ros-includes
#include<ros/ros.h>
#include <pcl_ros/point_cloud.h>


/**
   Subscribe to two point clouds, transform into 
   one frame. Merge them and then publish as a
   new Point Cloud
**/

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudT;

// callback:
void front_call(const PointCloudT::ConstPtr&);
void back_call(const PointCloudT::ConstPtr&);

//GLOBALs
//TODO: Get Rid of 'em
PointCloudT front_pc;
PointCloudT back_pc;
bool new_pc_f, new_pc_b;

using namespace std;

int main(int argc, char** argv)
{
  //ros
  ros::init(argc, argv, "merge_pc");
  ros::NodeHandle nh;
  ros::Publisher pc_pub = nh.advertise<PointCloudT> 
    ("/kinect_both/depth_registered/points", 1);
  ros::Subscriber front_sub = nh.subscribe<PointCloudT> 
    ("/kinect_front/depth_registered/points", 1, front_call);
  ros::Subscriber back_sub = nh.subscribe<PointCloudT> 
    ("/kinect_back/depth_registered/points", 1, back_call);

  //TODO: Change to a frame that is truly its
  string new_frame = "base_link";

  //Point Clouds
  PointCloudT pub_pc;

  //create a 3D trasnformation matrix 
  Eigen::Matrix4f TransMat; 
  TransMat <<         
    -0.7181,   -0.9601,   -0.1699,   -0.3431,
    0.2667,    0.4643,    0.8790,   -2.4817,
    -0.8105,    2.0802,    0.3348,    2.4483,
    0.0,         0.0,         0.0,    1.0000;

  //TODO: Change to Pointclouds from PointCloudTs
  //pcl::transformPointCloud(front_pc, pub_pc, TransMat ); 

  new_pc_f = false;
  new_pc_b = false;

  while(ros::ok()){
    ros::spinOnce();

    //if both PCs new then concatenate
    if (new_pc_f && new_pc_b){
      //concatenate
      pcl::transformPointCloud(front_pc, pub_pc, TransMat ); 
      pub_pc += back_pc;

      new_pc_f = false;
      new_pc_b = false;

      //publish
      pc_pub.publish(pub_pc);
    }

  }

  return 0;
}

// callback:
void front_call(const PointCloudT::ConstPtr& cloud)
{
  if (!new_pc_f){
    front_pc = *cloud;
    new_pc_f = true;
  }
    
}

void back_call(const PointCloudT::ConstPtr& cloud)
{
  if (!new_pc_b){
    back_pc = *cloud;
    new_pc_b = true;
  }
}
