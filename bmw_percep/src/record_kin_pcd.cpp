#include <pcl/conversions.h>
#include<iostream>
#include "ros/ros.h"
#include<bmw_percep/pcl_cv_utils.hpp>
#include<pcl/io/openni_grabber.h>
#include<opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sstream>
/**
   Accesses point clouds from a kinect and saves them into pcd files.
**/

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::mutex cloud_mutex;
enum { COLS=640, ROWS=480};

//Callback -- grabbing pointclouds from OpenNI
void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, 
		PointCloudT::Ptr& cloud,
		bool* new_cloud_available_flag)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from
			  // another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}

int main (int argc, char** argv)
{
  
  system("mkdir data/temp-pcd");
  string path="data/temp-pcd/"; string ext=".pcd"; string frame_file;
  
  //ros
  ros::init(argc, argv, "sample_tracker");
  ros::NodeHandle nh;

  //Various objects
  PointCloudT::Ptr cloud (new PointCloudT);
  cv::Mat rgb_im, depth_im, depth_mask;

  // Read Kinect live stream:
  bool new_cloud_available_flag = false;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
    boost::bind (&cloud_cb_, _1, cloud, &new_cloud_available_flag);
  interface->registerCallback (f);
  interface->start ();
  
  // Wait for the first frame:
  while(!new_cloud_available_flag) 
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  
  int nframes=0;
  char c=0;

  while(ros::ok()){
    if (new_cloud_available_flag && cloud_mutex.try_lock()){
      nframes++;
      cv_utils::pc_to_img(cloud, rgb_im, depth_im, depth_mask);

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
  }

  cv::destroyAllWindows();
  return 0;
}
