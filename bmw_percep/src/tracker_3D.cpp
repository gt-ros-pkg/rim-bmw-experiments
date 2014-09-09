#include<iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include<pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl/conversions.h>
#include<opencv2/opencv.hpp>
#include<bmw_percep/shr_cv_utils.hpp>
// #include<bmw_percep/ppl_detection.hpp>
#include<bmw_percep/groundPlane.hpp>
#include<bmw_percep/pplTrack.hpp>

//ros-includes
#include<ros/ros.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

/**
   Sample program for PCL based people detection
**/

typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PointCloudX;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// Globals
string hum_frame, robo_frame;
int frame_rate=30;
enum { COLS=640, ROWS=480};

//Various objects
PointCloudT::Ptr cloud (new PointCloudT);
//subscribes to the robots location
Eigen::Vector3f robo_loc;
  

bool new_cloud_available_flag;

//Pointcloud callback
void pc_call(const pcl::PCLPointCloud2 cloud);

// MAIN
int main(int argc, char** argv)
{
  string pub_topic = "/human/debug/pc";
  string file_topic = "/read_pcd"; 
  string live_bg_topic = "/subtracted_read_pcd";
  string live_topic = "/kinect_both/depth_registered/points";
  // string sub_topic = live_bg_topic;
  string sub_topic = live_topic;
  string viz_topic = "/human/visuals";
  string pkg_dir = "/home/shray/dev/hydro_ws/src/rim_bmw_experiments/bmw_percep/";
  robo_frame = "base_link";

  //ros
  ros::init(argc, argv, "sample_tracker");
  ros::NodeHandle nh;
// subscribe to input Point Cloud
  ros::Subscriber pc_sub = nh.subscribe<pcl::PCLPointCloud2> 
    (sub_topic, 1, pc_call);
//Publish the human clusters
  ros::Publisher db_pc = nh.advertise<pcl::PCLPointCloud2> (pub_topic, 1);
//Publish visualizations for the human
  ros::Publisher pub_viz = nh.advertise<visualization_msgs::MarkerArray> (viz_topic, 1);

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

  //People Tracker
  // PplTrack ppl_tracker(ground_coeffs);
  PplTrack ppl_tracker(0.15);
  bool ppl_frame_set = false;
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

    vector<vector<Eigen::Vector3f> > clusters;
    
    // ppl_detection::find_euclid_blobs(cloud, viz_cloud,  
    //     				clusters,
    //     				ground_coeffs);


    // //debug
    // if(viz_cloud->points.size()>0){
    // cout << "The numbero peeps is " << clusters.size();
    // string whap;
    // cin >> whap;
    // }

    //track
    ppl_tracker.estimate(cloud, 
			 clusters,
			 ground_coeffs,
			 robo_loc);

    // set frame-id if first frame
    if(!ppl_frame_set){
      ppl_tracker.set_viz_frame(hum_frame);
      ppl_frame_set=true;}
  
    ppl_tracker.visualize(pub_viz);

    // pcl::copyPointCloud(*cloud, *viz_cloud);
    if (cloud->points.size()>0){
      pcl::copyPointCloud(*cloud, *tcloud);
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
  //Subscribe tf
  tf::TransformListener tf_listener;

  //cout << "Frame = " << temp_cloud.header.frame_id << endl;
  hum_frame = temp_cloud.header.frame_id;
  pcl::fromPCLPointCloud2(temp_cloud, *cloud);
  
  // //also listen to robot--right now
  // tf::StampedTransform robo_form;
  // try{
  //   tf_listener.waitForTransform(robo_frame, hum_frame, ros::Time(0),
  // 				 ros::Duration(1.5));
  //   tf_listener.lookupTransform(robo_frame, hum_frame, ros::Time(0),
  // 				      robo_form);
  // }
  // catch(tf::TransformException &ex){
  //   cout << ex.what() << endl;
  //   ROS_ERROR("%s", ex.what());
  //   return;
  // }

  // Eigen::Vector3d temp_vec;
  // tf::vectorTFToEigen(robo_form.getOrigin(), temp_vec);
  // robo_loc = temp_vec.cast<float>();
  // cout << "Get here?" << endl;
  new_cloud_available_flag = true;
}
