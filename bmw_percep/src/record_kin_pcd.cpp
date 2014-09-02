#include <pcl/conversions.h>
#include <iostream>
#include "ros/ros.h"
#include <bmw_percep/shr_cv_utils.hpp>
#include <pcl/io/openni_grabber.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sstream>
#include <pcl_ros/point_cloud.h>

/**
   Accesses point clouds from a ROS topic and saves them into pcd files.
**/

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// GLOBALS
boost::mutex cloud_mutex;
enum { COLS=640, ROWS=480};
//Various objects
PointCloudT::Ptr cloud (new PointCloudT);
cv::Mat rgb_im, depth_im, depth_mask;
bool new_cloud_available_flag;

//Pointcloud callback
void pc_call(const pcl::PCLPointCloud2 cloud);

//MAIN
int main (int argc, char** argv)
{
  system("mkdir data/temp-pcd");
  string path="data/temp-pcd/"; string ext=".pcd"; string frame_file;

  string front_topic = "/kinect_front/depth_registered/points";
  string back_topic = "/kinect_back/depth_registered/points";

  //ros
  ros::init(argc, argv, "sample_tracker");
  ros::NodeHandle nh;
  ros::Subscriber pc_sub = nh.subscribe<pcl::PCLPointCloud2> 
    (back_topic, 1, pc_call);


  int nframes=0;
  char c=0;

  while(ros::ok()){
    ros::spinOnce();
    if (new_cloud_available_flag)
      nframes++;
    else
      continue;
    shr_cv_utils::pc_to_img(cloud, rgb_im, depth_im, depth_mask);
    cv::imshow("RGB image", rgb_im);
    cv::imshow("Depth image", depth_im);
      c = cv::waitKey(5);
      if (c==27){break;}
      //write PCD
      ostringstream convertToStr;
      convertToStr << nframes;
      frame_file = path + convertToStr.str() + ext;
      //debug
      cout << "Save me! "<< frame_file << endl;
      pcl::io::savePCDFileBinary(frame_file, *cloud);
      cloud_mutex.unlock();
      //int a =1;
      //new_cloud_available_flag = true;
      cout << "\n Continue On.." << endl;
      //continue;
      //debug
      //if (nframes>100)
      //break;
  }
cv::destroyAllWindows();
  return 0;
}

// callback:
void pc_call(const pcl::PCLPointCloud2 temp_cloud)
{
  pcl::fromPCLPointCloud2(temp_cloud, *cloud);
    new_cloud_available_flag = true;
}
