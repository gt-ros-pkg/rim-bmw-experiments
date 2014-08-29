#include<bmw_percep/groundPlane.hpp>
#include<iostream>
#include<pcl/io/openni_grabber.h>

//ros-includes
#include<ros/ros.h>
#include <pcl_ros/point_cloud.h>

/**
   Sample program testing the background subtraction after subscribing 
   to a PointCloud.
**/

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//GLOBALS
boost::mutex cloud_mutex;
enum { COLS=640, ROWS=480};
PointCloudT::Ptr global_cloud (new PointCloudT);
bool new_cloud_available_flag;

void pc_call(const pcl::PCLPointCloud2);

// //Callback -- grabbing pointclouds from OpenNI
// void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, 
// 		PointCloudT::Ptr& cloud,
// 		bool* new_cloud_available_flag)
// {
//   cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
//   *cloud = *callback_cloud;
//   *new_cloud_available_flag = true;
//   cloud_mutex.unlock ();
// }
    
int main(int argc, char** argv)
{

string front_topic = "/kinect_front/depth_registered/points";
string back_topic = "/kinect_back/depth_registered/points";

//ros
  ros::init(argc, argv, "sample_ground");
  ros::NodeHandle nh;
  // ros::Subscriber pc_sub = nh.subscribe<PointCloudT> 
  //   (back_topic, 1, pc_call);

  ros::Subscriber pc_sub = nh.subscribe<pcl::PCLPointCloud2> 
    (back_topic, 1, pc_call);

  // // Read Kinect live stream:
  // PointCloudT::Ptr cloud (new PointCloudT);
  // bool new_cloud_available_flag = false;
  // pcl::Grabber* interface = new pcl::OpenNIGrabber();
  // boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
  //   boost::bind (&cloud_cb_, _1, cloud, &new_cloud_available_flag);
  // interface->registerCallback (f);
  // interface->start ();

  new_cloud_available_flag = false;
  ros::spin();

  // Wait for the first frame:
  while(!new_cloud_available_flag) 
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  new_cloud_available_flag = false;

  cloud_mutex.lock ();    // for not overwriting the point cloud

  //debug
  cout << endl << "Gets here?" << endl;
  GroundPlane ground_obj(global_cloud);
  
  //write these to file?
  char yn;
  cout << endl << "Write coefficients to file:(y/n)<return>"; 
  cin >> yn; cout << endl;
  if (yn=='y'){
    //choose file name
    string path = "data/";
    string fileName = "ground_coeffs.txt";
    cout << endl << "file name : " << fileName << " correct? (y/n)<return>"; 
    cin>>yn; cout << endl;
    if(yn!='y'){
      cout << "New name: "; cin >> fileName; cout << endl;
    }
    string total_file = path + fileName;
    ground_obj.writeFile(total_file);
  }

  cloud_mutex.unlock();

  cout << "Locked no more!" << endl;

  return 0;
}

// callback:
void pc_call(const pcl::PCLPointCloud2 cloud)
{
  cloud_mutex.lock();
  //global_cloud = cloud;
// pcl::copyPointCloud(*cloud, *global_cloud);
//global_cloud.reset(new PointCloudT (*cloud));
//pcl::PointCloud<PointXYZ>(*cloud)); 
  pcl::fromPCLPointCloud2(cloud, *global_cloud);
new_cloud_available_flag = true;
cloud_mutex.unlock();
}
