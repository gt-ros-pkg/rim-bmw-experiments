#include<iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl/conversions.h>
#include<opencv2/opencv.hpp>
#include<bmw_percep/pcl_cv_utils.hpp>
#include<bmw_percep/groundPlane.hpp>

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

enum { COLS=640, ROWS=480};

cv::Mat learn_background(const string backg_folder);

// MAIN
int main(int argc, char** argv)
{
  //ros
  ros::init(argc, argv, "sample_tracker");
  ros::NodeHandle nh;
  ros::Publisher db_pc = nh.advertise<pcl::PCLPointCloud2> ("/human/debug/pc", 1);

  string hum_frame = "base_link";
  string read_dir = "/home/menchi/dev/shray-hydro-ws/src/ppl_navigate/data/pcd/sequences1/9/";
  //  string read_dir = "src/ppl_navigate/data/pcd/temp1/"; 
  string read_file, write_file;
  string write_dir = read_dir;
  string pcd_ext = ".pcd";

  //background
  string backg_folder = "/home/menchi/dev/shray-hydro-ws/src/ppl_navigate/data/pcd/sequences1/background/";
  read_dir = backg_folder;
  cv::Mat backg_img = learn_background(backg_folder);

  cv::namedWindow("BackG",2);
  cv::imshow("BackG", backg_img);
  cv::waitKey(0);
  
  PointCloudT::Ptr cloud (new PointCloudT);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    tcloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCLPointCloud2 pub_pc;
      
  cv::Mat rgb_im, depth_im, valid_depth, foreMask, back_im, disp_im, box_mask, box_mask2, box_mask_inliers;

  //ground-plane
  Eigen::VectorXf ground_coeffs;
  //read ground plane parameters from file
  string fileName = "data/ground_coeffs.txt";
  GroundPlane ground_obj(fileName);
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
    cv_utils::find_euclid_blobs(cloud, viz_cloud,  
				  clusters, max_blob_id,
				ground_coeffs, backg_img);

      pcl::copyPointCloud(*viz_cloud, *tcloud);
      pcl::toPCLPointCloud2(*tcloud, pub_pc);
      pub_pc.header.frame_id = hum_frame;
      db_pc.publish(pub_pc);

      int useless;
      //cin >> useless;
      // cv_utils::pc_to_img(cloud, rgb_im, depth_im, valid_depth);
      // cv::imshow("rgb", rgb_im);
      // cv::waitKey(10);
      //boost::this_thread::sleep(boost::posix_time::milliseconds(200));

  }

  return 0;
}

cv::Mat learn_background(const string backg_folder)
{
  int frame_no=0;
  cv::Mat rgb_im, depth_im, depth_mask, foreMask, mean_img, frame_mat;
  PointCloudT::Ptr cloud (new PointCloudT);

  while(ros::ok())
    {

      frame_no++;
      ostringstream read_fr_str;
      read_fr_str << frame_no;
      string read_file = backg_folder + read_fr_str.str() + ".pcd";
    
      if (pcl::io::loadPCDFile<PointT> (read_file, *cloud) == -1){
      	cout << "\nRead " << frame_no-1 << " background frames .." << endl;
	break;
      }
      
      else{
      	//file read learn it!
	cv_utils::pc_to_img(cloud, rgb_im, depth_im, depth_mask);
	if (frame_no==1){
	  mean_img = cv::Mat::zeros(cloud->height, cloud->width, CV_64F);
	  frame_mat =cv::Mat::zeros(cloud->height, cloud->width, CV_64F);
	}
	//cv::imshow("Background learning", depth_im);
	//cv::waitKey(5);
	// mean_img = mean_img + depth_im;
	for (int r=0; r<cloud->height; r++){

	  double* mean_r = mean_img.ptr<double> (r);
	  double* frame_r = frame_mat.ptr<double> (r);
	  
	  for (int c=0; c<cloud->width; c++){
	    PointT point = cloud->at(c,r);
	    //set depth and depth-mask if not NaN
	    if (!isnan(point.z) && (point.z<6.0 && point.z>0.5)){
	      mean_r[c] = (double)point.z;
	      frame_r[c] ++;
	    }
	  }
	}
      }
    }
  
  cv::Mat bg_im;
  // bg_im = mean_img;
  for (int r=0; r<cloud->height; r++){
    double* mean_r = mean_img.ptr<double> (r);
    double* frame_r = frame_mat.ptr<double> (r);
    for (int c=0; c<cloud->width; c++){
      if (frame_r[c]>0){      
	mean_r[c] /= frame_r[c];
      }
      else{
	mean_r[c] = 0;
      }
    }
  }

  bg_im = mean_img;
  return bg_im;
}
