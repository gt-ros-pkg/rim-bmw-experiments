#include<iostream>
// #include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// #include<pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include<pcl/conversions.h>
#include<opencv2/opencv.hpp>
#include<bmw_percep/shr_cv_utils.hpp>
// #include<bmw_percep/ppl_detection.hpp>
#include<bmw_percep/groundPlane.hpp>
#include<bmw_percep/kalmanFilter.hpp>
#include<bmw_percep/pplTrack.hpp>
#include <boost/timer.hpp>

//ros-includes
#include<ros/ros.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

/**
   Sample program for PCL based people detection
**/

typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PointCloudX;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

enum CamMode {BOTH, FRONT, BACK};

class Tracker3d{

private:
  ros::NodeHandle nh_;
  // Globals
  ros::Time cur_pc_time;
  string hum_frame, robo_frame;
  int frame_rate;
  CamMode kin_mode;

enum { COLS=640, ROWS=480};
  //Subscribe tf
  tf::TransformListener tf_listener;

//Various objects
  PointCloudT::Ptr cloud;
  //subscribes to the robots location
  Eigen::Vector3f robo_loc;

bool got_transform_;
  bool new_cloud_available_flag;

  ros::Subscriber pc_sub;
  ros::Subscriber pc_sub_;
  ros::Publisher db_pc;
  ros::Publisher pub_viz;
  ros::Publisher pub_raw_obs;
  ros::Publisher  pub_obs_pos;
  ros::Publisher pub_est_pos;
  ros::Publisher pub_debug_pos;
  ros::Publisher pub_hum_work;
  ros::Publisher pub_status_msg;

protected:
  //Pointcloud callback
  void pc_call(const PointCloudT::ConstPtr& );
  void recvPCCallback(const PCRGB::ConstPtr& );

public:
  Tracker3d(ros::NodeHandle& nh);

};

// MAIN
Tracker3d::Tracker3d(ros::NodeHandle& nh):cloud(new PointCloudT), nh_(nh), 
					  kin_mode(BOTH)
{

  got_transform_ = false;
  frame_rate=30;
  string pub_topic = "/human/debug/pc";
  string file_topic = "/read_pcd";
  string live_bg_topic = "/subtracted_read_pcd";
  string temp_topic = "/kinect_back/world/depth_registered/points";
  string live_topic = "/kinect_both/depth_registered/points";
  string back_topic ="/kinect_back/world/depth_registered/points"; 
  string front_topic ="/kinect_front/world/depth_registered/points"; 

  string sub_topic = live_topic;
  // string sub_topic = file_topic;
  string viz_topic = "/human/visuals";
  string raw_obs_topic = "/human/observations/visuals";
  string pkg_dir = "/home/shray/dev/hydro_ws/src/rim_bmw_experiments/bmw_percep/";
  robo_frame = "base_link";
  string obs_pose_topic = "/human/observations/position";
  string est_pose_topic = "/human/estimated/pose";
  string debug_pos_topic = "/debug/pose";
  string hum_work_topic = "/human/workspace/occupied";
  string status_topic = "/display/message";

  pc_sub = nh.subscribe<PointCloudT> 
    (sub_topic, 1, &Tracker3d::pc_call, this);

  //Publish the human clusters
  db_pc = nh.advertise<pcl::PCLPointCloud2> (pub_topic, 1);
  //Publish visualizations for the human
  pub_viz = nh.advertise<visualization_msgs::MarkerArray> (viz_topic, 1);
  pub_raw_obs = nh.advertise<visualization_msgs::MarkerArray> (raw_obs_topic, 1);
  pub_obs_pos = nh.advertise<geometry_msgs::PoseArray> (obs_pose_topic, 1);
  pub_est_pos = nh.advertise<geometry_msgs::PoseArray> (est_pose_topic, 1);
  pub_debug_pos = nh.advertise<geometry_msgs::PoseStamped> (debug_pos_topic, 1);
  pub_hum_work = nh.advertise<std_msgs::Bool>(hum_work_topic, 1);
  pub_status_msg = nh.advertise<std_msgs::String>(status_topic, 1);

  PointCloudT::Ptr
    tcloud (new PointCloudT);
  pcl::PCLPointCloud2 pub_pc;

  //People Tracker
  float ground_coeffs=.25;
  // PplTrack ppl_tracker(ground_coeffs);
  PplTrack ppl_tracker(ground_coeffs);
  bool ppl_frame_set = false;
  int n_frames=0;
  
  bool done_reading_files = false;
  bool reached = false;
  PointCloudT::Ptr 
    viz_cloud (new PointCloudT);


  boost::timer time_since_cloud;
  double exit_both=60.; // time to wait until both mode is exited
  double switch_kin=5.; // time to wait before switching to other kinect

  while(ros::ok()){
    // //debug
    // cout << "Got to the while? " << endl;
    new_cloud_available_flag=false;
    ros::spinOnce();
    // cout << "Spun" << endl;
    if (new_cloud_available_flag)
      n_frames++;
    else{ //if no clouds - check timer
      if (kin_mode == BOTH && time_since_cloud.elapsed()>exit_both){
	pc_sub.shutdown();
	pc_sub = nh.subscribe<PointCloudT> 
	  (front_topic, 1, &Tracker3d::pc_call, this);
	time_since_cloud.restart();
      }
      else if(kin_mode==FRONT && time_since_cloud.elapsed()>switch_kin){
	pc_sub.shutdown();
	pc_sub = nh.subscribe<PointCloudT> 
	  (back_topic, 1, &Tracker3d::pc_call, this);
	time_since_cloud.restart();
      }
      else if(kin_mode==BACK && time_since_cloud.elapsed()>switch_kin){
	pc_sub.shutdown();
	pc_sub = nh.subscribe<PointCloudT> 
	  (front_topic, 1, &Tracker3d::pc_call, this);
	time_since_cloud.restart();
      }
      continue;
    }
    vector<vector<Eigen::Vector3f> > clusters;
    
    //check if cloud empty
    if (cloud->points.size()<1)
      continue;
    //track
    ppl_tracker.estimate(cloud, 
			 clusters,
			 robo_loc, got_transform_,
			 cur_pc_time);

    // set frame-id if first frame
    if(!ppl_frame_set){
      ppl_tracker.set_viz_frame(hum_frame);
      ppl_frame_set=true;}
  
    // ppl_tracker.visualize(pub_viz);
    ppl_tracker.visualize_est(pub_viz);
    ppl_tracker.visualize_obs(pub_raw_obs);

    ppl_tracker.pub_obs_est(pub_obs_pos, pub_est_pos);
    ppl_tracker.pub_human_workspace(pub_hum_work, pub_status_msg);

    // pcl::copyPointCloud(*cloud, *viz_cloud);
    if (cloud->points.size()>0){
      pcl::copyPointCloud(*cloud, *tcloud);
      pcl::toPCLPointCloud2(*tcloud, pub_pc);
      pub_pc.header.frame_id = hum_frame;
      db_pc.publish(pub_pc);

      //debug
      // cout << "Found a person, Aaaaah!" << endl;
      // string useless; cin >> useless;

    }
    // int useless;
    //cin >> useless;
    // cv_utils::pc_to_img(cloud, rgb_im, depth_im, valid_depth);
    // cv::imshow("rgb", rgb_im);
    // cv::waitKey(10);
    //boost::this_thread::sleep(boost::posix_time::milliseconds(200));
  }

  return;
}

// callback:
void Tracker3d::pc_call(const PointCloudT::ConstPtr& temp_cloud)
{

  hum_frame = temp_cloud->header.frame_id;

  boost::timer timer_total, timer_tf;

  // cout << "Frame = " << temp_cloud->header.frame_id << endl;
  
  pcl::copyPointCloud(*temp_cloud, *cloud);

  std_msgs::Header tmp_head = pcl_conversions::fromPCL(cloud->header);
  cur_pc_time = tmp_head.stamp; //ros::Time::now();


  new_cloud_available_flag = true;
  
  //also listen to robot--right now
  tf::StampedTransform robo_form;
  try{
    ros::Time a = ros::Time(0);
    // tf_listener.waitForTransform(hum_frame, robo_frame, a,
    // 				 ros::Duration(1.0));
    tf_listener.lookupTransform(hum_frame, robo_frame, a,
  				      robo_form);
  }
  catch(tf::TransformException &ex){
    cout << ex.what() << endl;
    ROS_ERROR("%s", ex.what());
    got_transform_ = false;
    return;
  }

  got_transform_ = true;

  Eigen::Vector3d temp_vec;
  tf::vectorTFToEigen(robo_form.getOrigin(), temp_vec);
  robo_loc = temp_vec.cast<float>();
  // //debug
  // cout << "Get here? Robot location: " << robo_loc <<  endl;

  float time_tf = timer_tf.elapsed();

  float time_total = timer_total.elapsed();
  
  //debug
  cout << "**********TIMES**********\n";
  cout << "Time TF = " << time_tf << '\n';
  cout << "Time Total = " << time_total << endl;
}

void Tracker3d::recvPCCallback(const PCRGB::ConstPtr& pc_in){
}
