#include<iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl/conversions.h>
#include<opencv2/opencv.hpp>
#include<bmw_percep/pcl_cv_utils.hpp>

//ros-includes
#include<ros/ros.h>

/**
   Sample program for PCL based people detection
**/

typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PointCloudX;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Globals
int frame_rate=30;

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

// MAIN
int main(int argc, char** argv)
{
  //ros
  ros::init(argc, argv, "sample_tracker");
  ros::NodeHandle nh;
  ros::Publisher db_pc = nh.advertise<pcl::PCLPointCloud2> ("/human/debug/pc", 1);

  string hum_frame = "base_link";

  PointCloudT::Ptr cloud (new PointCloudT);
  pcl::PCLPointCloud2 pub_pc;
      
  cv::Mat depth_im, valid_depth, foreMask, back_im, disp_im, box_mask, box_mask2, box_mask_inliers;

  // Read Kinect live stream:
  bool new_cloud_available_flag = false;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  boost::function<void (const PointCloudT::ConstPtr&)> f =
    boost::bind (&cloud_cb_, _1, cloud, &new_cloud_available_flag);
  interface->registerCallback (f);
  interface->start ();

  // Wait for the first frame:
  while(!new_cloud_available_flag) 
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));

  int n_frames=0;
  
  while(ros::ok()){
    if (new_cloud_available_flag && cloud_mutex.try_lock()){

      new_cloud_available_flag = false;
      n_frames++;

      PointCloudT::Ptr 
	viz_cloud (new PointCloudT);

      vector<cv::Point3f> clusters; int max_blob_id;
      //cv_utils::find_euclid_blobs(cloud, viz_cloud, foreMask, 
				  // clusters, max_blob_id);
      pcl::toPCLPointCloud2(*viz_cloud, pub_pc);
      // pcl::toPCLPointCloud2(*cloud, pub_pc);
      pub_pc.header.frame_id = hum_frame;
      db_pc.publish(pub_pc);



    }

  }
  return 0;
}
