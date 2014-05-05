#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <opencv2/opencv.hpp>
#include <bmw_percep/colorHistogram.hpp>
#include <bmw_percep/pcl_cv_utils.hpp>

//ros-includes
#include<ros/ros.h>

/**
   Sample program for learning and storing a color histogram 
   from a series of point clouds
**/

int main(int argc, char** argv)
{

  //ros
  ros::init(argc, argv, "color_hist");
  ros::NodeHandle nh;


  ColorHistogram robo_hist(256);
  
  string read_dir = "data/PCDs/robot2/", pcd_ext=".pcd";
  PointCloudT::Ptr cloud (new PointCloudT);
  cv::Mat rgb_im, depth_im, depth_mask, fore, fore_hist;
  string read_file;
  int n_frames=0;
  bool done_reading_files=false;
  char c;
  int win_f=30;

  while(ros::ok() && c!=27){
    n_frames++;

    ostringstream read_fr_str;
    read_fr_str << n_frames;
    read_file = read_dir + read_fr_str.str() + pcd_ext;
  
    if (!done_reading_files){
      if (pcl::io::loadPCDFile<PointT> (read_file, *cloud) == -1 ){
	//done reading
	cout << "\n All files read. I guess." << endl;
	n_frames=0;
	break;
      }
    }

    cv_utils::pc_to_img_no_filter(cloud, rgb_im, depth_im, depth_mask);
    
    if (n_frames==1){fore.create(rgb_im.size(), CV_8UC3);}

    fore = cv::Scalar(0);
    
    for(int r=0; r < depth_im.rows; r++){

      if (r<win_f || r>(depth_im.rows-win_f))
	continue;
      double* depth_r = depth_im.ptr<double> (r);
      uchar* dmask_r = depth_mask.ptr<uchar> (r);
      cv::Vec3b* rgb_r = rgb_im.ptr<cv::Vec3b> (r);
      cv::Vec3b* fore_r = fore.ptr<cv::Vec3b> (r);

      for(int c=0; c<depth_im.cols; c++){
	if (c>win_f && c<(depth_im.cols-win_f)){
	  if (depth_r[c]<2.0 || isnan(depth_r[c])){
	    fore_r[c][0] = rgb_r[c][0];
	    fore_r[c][1] = rgb_r[c][1];
	    fore_r[c][2] = rgb_r[c][2];
	  }
	}
      }
    }
    
    robo_hist.addToObj(fore);
    // cv::imshow("Robo-man", fore);
    // c = cv::waitKey(5);
  }

  // learn background histogram
  string backg_dir = "/home/menchi/dev/shray-hydro-ws/src/ppl_navigate/data/pcd/sequences1/background/";
  bool done_backg = false;
  int backg_frame = 0;
  
  while(!done_backg){
    backg_frame++;
    ostringstream read_fr_str;
    read_fr_str << backg_frame;
    string read_file = backg_dir + read_fr_str.str() + pcd_ext;
    
    if (pcl::io::loadPCDFile<PointT> (read_file, *cloud) == -1 ){
      //done reading
      cout << "\n All files read. I guess." << endl;
      done_backg = true;
      break;
    }

    cv_utils::pc_to_img_no_filter(cloud, rgb_im, depth_im, depth_mask);
    robo_hist.addToBackground(rgb_im);
    
  }

  robo_hist.displayHists();

  // test histogram
  bool done_test=false;
  int test_frame=0;
  fore_hist.create(rgb_im.size(), CV_8UC1);
  while(!done_test){
    test_frame++;
    ostringstream read_fr_str;
    read_fr_str << backg_frame;
    string read_file = read_dir + read_fr_str.str() + pcd_ext;
    
    if (pcl::io::loadPCDFile<PointT> (read_file, *cloud) == -1 ){
      //done reading
      cout << "\n All files read. I guess." << endl;
      done_test = true;
      break;
    }

    cv_utils::pc_to_img_no_filter(cloud, rgb_im, depth_im, depth_mask);
    robo_hist.testImg(rgb_im, fore_hist);
    //debug
    c = cv::waitKey(5);
    if (c==27)
      break;
  }


  return 0;
}
