#include <pcl/conversions.h>
#include<iostream>
#include "ros/ros.h"
#include<bmw_percep/pcl_cv_utils.hpp>
#include<pcl/io/openni_grabber.h>
#include<opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

/**
   Publishes the following from a connected RGBD sensor:
   - RGB image
   - Depth Map (with invalid pixels as zeros)
   - Depth Mask
   - PointCloud
**/

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


int main(int argc, char** argv)
{
  //TODO: Change the transports to image transports..
  //TODO: Write message headers

  //ros
  ros::init(argc, argv, "kinect_msg_publisher");
  ros::NodeHandle nh;

  // Create a ROS publishers
  ros::Publisher pub_cb = nh.advertise<sensor_msgs::PointCloud2> ("/kinect/cloud", 1);
  ros::Publisher pub_rgb = nh.advertise<sensor_msgs::Image> ("/kinect/rgb_image", 1);
  ros::Publisher pub_depth = nh.advertise<sensor_msgs::Image> ("/kinect/depth_map", 1);
  ros::Publisher pub_depth_mask = nh.advertise<sensor_msgs::Image> ("/kinect/mask", 1);
  
  //convert to Image messages
  cv_bridge::CvImage rgb_br;
  cv_bridge::CvImage depth_br;
  cv_bridge::CvImage mask_br;

  // write correct encodings
  rgb_br.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  depth_br.encoding = sensor_msgs::image_encodings::TYPE_64FC1;
  mask_br.encoding = sensor_msgs::image_encodings::TYPE_8UC1;

  //Various objects
  PointCloudT::Ptr cloud (new PointCloudT);
  cv::Mat rgb_im, depth_im, valid_depth;

  rgb_br.image = rgb_im;
  depth_br.image = depth_im;
  mask_br.image = valid_depth;


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
  new_cloud_available_flag = false;

  for (;;){
    cloud_mutex.lock ();    // for not overwriting the point cloud
    // chain of events start
   
    //get image, depth-map, valid mask
    bool yo = cv_utils::pc_to_img(cloud, rgb_im, depth_im, valid_depth);
    
    if (yo){
      //pub_cb.publish(cloud);
      pub_rgb.publish(rgb_br.toImageMsg());
      pub_depth.publish(depth_br.toImageMsg());
      pub_depth_mask.publish(mask_br.toImageMsg());
    }
    // cv::imshow("Color Image", rgb_im);
    // char c = cv::waitKey(15);
    
    cloud_mutex.unlock (); // let go of current cloud
  }

  return 0; // successfully exit
}
