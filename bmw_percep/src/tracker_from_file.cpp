#include<iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include<pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl/conversions.h>
#include<opencv2/opencv.hpp>
#include<bmw_percep/shr_cv_utils.hpp>
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
int frame_rate=30;

enum { COLS=640, ROWS=480};

// MAIN
int main(int argc, char** argv)
{
  //ros
  ros::init(argc, argv, "sample_tracker");
  ros::NodeHandle nh;
  ros::Publisher db_pc = nh.advertise<pcl::PCLPointCloud2> ("/human/debug/pc", 1);

  string hum_frame = "kinect_back_rgb_optical_frame";
  // string read_dir = "/home/menchi/dev/shray-hydro-ws/src/ppl_navigate/data/pcd/sequences1/9/";
  string pkg_dir = "/home/shray/dev/hydro_ws/src/rim_bmw_experiments/bmw_percep/";
  string read_dir = pkg_dir + "data/test_seq_1/";
  //  string read_dir = "src/ppl_navigate/data/pcd/temp1/"; 
  string read_file;
  string pcd_ext = ".pcd";

  PointCloudT::Ptr cloud (new PointCloudT);
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

    vector<cv::Point3f> clusters; int max_blob_id;
    
    // cv_utils::find_euclid_blobs(cloud, viz_cloud,  
    // 				clusters, max_blob_id,
    // 				ground_coeffs, cvBg);


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
