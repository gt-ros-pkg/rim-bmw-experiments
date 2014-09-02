#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <opencv2/opencv.hpp>

//ros-includes
#include<ros/ros.h>

/**
   Program to read PCD files from a folder and publish on a ROS topic.
**/

typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PointCloudX;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;


using namespace std;

// Globals
int frame_rate=30;

enum { COLS=640, ROWS=480};

// MAIN
int main(int argc, char** argv)
{
  //ros
  ros::init(argc, argv, "pcd_pub");
  ros::NodeHandle nh;
  ros::Publisher db_pc = nh.advertise<pcl::PCLPointCloud2> ("/read_pcd/", 1);

  string hum_frame = "kinect_back_rgb_optical_frame";
  // string read_dir = "/home/menchi/dev/shray-hydro-ws/src/ppl_navigate/data/pcd/sequences1/9/";
  string pkg_dir = "/home/shray/dev/hydro_ws/src/rim_bmw_experiments/bmw_percep/";
  string read_dir = pkg_dir + "data/test_seq_1/";
  //  string read_dir = "src/ppl_navigate/data/pcd/temp1/"; 
  string read_file;
  string pcd_ext = ".pcd";

  PointCloudT::Ptr cloud (new PointCloudT);
  pcl::PCLPointCloud2 pub_pc;
      
  int n_frames=0;
  
  bool done_reading_files = false;
  bool reached = false;

  while(ros::ok()){
    n_frames++;

    ostringstream read_fr_str;
    read_fr_str << n_frames;
    read_file = read_dir + read_fr_str.str() + pcd_ext;
  
    if (!done_reading_files){
      if (pcl::io::loadPCDFile<PointT> (read_file, *cloud) == -1 ){
      //done reading
      cout << "\n All files read. I guess." << endl;
      n_frames=0;
      //break;
      }
    }

    if (cloud->points.size()>0){
      // pcl::copyPointCloud(*viz_cloud, *tcloud);
      pcl::toPCLPointCloud2(*cloud, pub_pc);
      pub_pc.header.frame_id = hum_frame;
      db_pc.publish(pub_pc);
    }
  }

  return 0;
}
