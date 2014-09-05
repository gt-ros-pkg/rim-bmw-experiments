#include<iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include<pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl/conversions.h>
#include<opencv2/opencv.hpp>
#include<bmw_percep/shr_cv_utils.hpp>
#include<bmw_percep/ppl_detection.hpp>
#include<bmw_percep/groundPlane.hpp>

//ros-includes
#include<ros/ros.h>

/**
   Sample program for PCL based people detection
**/

typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PointCloudX;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Globals
string hum_frame;
int frame_rate=30;
enum { COLS=640, ROWS=480};

//Various objects
PointCloudT::Ptr cloud (new PointCloudT);

bool new_cloud_available_flag;

//Pointcloud callback
void pc_call(const pcl::PCLPointCloud2 cloud);

// MAIN
int main(int argc, char** argv)
{
  string pub_topic = "/human/debug/pc";
  string file_topic = "/read_pcd"; 
  string live_bg_topic = "/subtracted_read_pcd";
  // string sub_topic = live_bg_topic;
  string sub_topic = file_topic;
  string pkg_dir = "/home/shray/dev/hydro_ws/src/rim_bmw_experiments/bmw_percep/";

  //ros
  ros::init(argc, argv, "sample_tracker");
  ros::NodeHandle nh;
  ros::Subscriber pc_sub = nh.subscribe<pcl::PCLPointCloud2> 
    (sub_topic, 1, pc_call);
  ros::Publisher db_pc = nh.advertise<pcl::PCLPointCloud2> (pub_topic, 1);

  // PointCloudT::Ptr cloud (new PointCloudT);
  PointCloudT::Ptr
    tcloud (new PointCloudT);
  pcl::PCLPointCloud2 pub_pc;
  // cv::Mat rgb_im, depth_im, valid_depth, foreMask, back_im, disp_im, box_mask, box_mask2, box_mask_inliers;

  //ground-plane
  Eigen::VectorXf ground_coeffs;
  //read ground plane parameters from file
  string gr_file_name = pkg_dir + "data/ground_coeffs.txt";
  GroundPlane ground_obj(gr_file_name);
  ground_obj.get_ground_coeffs(ground_coeffs);

  int n_frames=0;
  
  bool done_reading_files = false;
  bool reached = false;
  PointCloudT::Ptr 
    viz_cloud (new PointCloudT);

  while(ros::ok()){
    ros::spinOnce();
    if (new_cloud_available_flag)
      n_frames++;
    else
      continue;

    vector<cv::Point3f> clusters; int max_blob_id;
    
    
    ppl_detection::find_euclid_blobs(cloud, viz_cloud,  
    				clusters, max_blob_id,
    				ground_coeffs);


    // pcl::copyPointCloud(*cloud, *viz_cloud);
    if (viz_cloud->points.size()>0){
      pcl::copyPointCloud(*viz_cloud, *tcloud);
      pcl::toPCLPointCloud2(*tcloud, pub_pc);
      pub_pc.header.frame_id = hum_frame;
      db_pc.publish(pub_pc);
    }
      int useless;
      //cin >> useless;
      // cv_utils::pc_to_img(cloud, rgb_im, depth_im, valid_depth);
      // cv::imshow("rgb", rgb_im);
      // cv::waitKey(10);
      //boost::this_thread::sleep(boost::posix_time::milliseconds(200));
  }

  return 0;
}

// callback:
void pc_call(const pcl::PCLPointCloud2 temp_cloud)
{
  //cout << "Frame = " << temp_cloud.header.frame_id << endl;
  hum_frame = temp_cloud.header.frame_id;
  pcl::fromPCLPointCloud2(temp_cloud, *cloud);
    new_cloud_available_flag = true;
}
