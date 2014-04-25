#include<bmw_percep/groundPlane.hpp>
#include<iostream>
#include<pcl/io/openni_grabber.h>
#include<ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
/**
   Sample program testing the background subtraction after connecting to 
   single RGBD camera.
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
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}

int main(int argc, char** argv)
{

  //ros
  ros::init(argc, argv, "ground_projected");
  ros::NodeHandle nh;

  // Create a ROS publishers
  ros::Publisher pub_cb = nh.advertise<sensor_msgs::PointCloud2> ("/ground/project", 1);
  pcl::PCLPointCloud2 cloud_pub;

  //read ground plane parameters from file
  string fileName = "data/ground_coeffs.txt";
  //debug
  cout << "\nMade it here" << endl;

  GroundPlane ground_obj(fileName);

  //debug
  cout << "\n Ground plane got." << endl;
  // Read Kinect live stream:
  PointCloudT::Ptr cloud (new PointCloudT);
  PointCloudT::Ptr cloud_projected (new PointCloudT);

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

  cloud_mutex.lock ();    // for not overwriting the point cloud

  //visualize plane on rgb-imagery
  ground_obj.visualizePlane(cloud, 0.1);


  cloud_mutex.unlock();

  while(ros::ok()){
   if (new_cloud_available_flag && cloud_mutex.try_lock()){
     new_cloud_available_flag = false;

     //project plane
     ground_obj.pcProject(cloud, cloud_projected);
     
     //publish
     //cloud_pub.header.stamp = ros::Time::now();
     cloud_pub.header.frame_id = "sensor_frame";
     //pcl::toROSMsg(*cloud, cloud_pub);
     pub_cb.publish(cloud_pub);
	    
     
     cloud_mutex.unlock();
   }   
  }

  cout << "Locked no more!" << endl;

  return 0;
}
