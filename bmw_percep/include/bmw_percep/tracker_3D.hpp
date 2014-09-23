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

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

class Tracker3d{

private:
  ros::NodeHandle nh_;
  // Globals
  string hum_frame, robo_frame;
  int frame_rate;
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

protected:
  //Pointcloud callback
  void pc_call(const PointCloudT::ConstPtr& );
  void recvPCCallback(const PCRGB::ConstPtr& );

public:
  Tracker3d(ros::NodeHandle& nh);

};

// MAIN
Tracker3d::Tracker3d(ros::NodeHandle& nh):cloud(new PointCloudT), nh_(nh)
{

got_transform_ = false;
  //debug
  cout << "Are you even compiling?" << endl;
  frame_rate=30;
  string pub_topic = "/human/debug/pc";
  string file_topic = "/read_pcd";
  string live_bg_topic = "/subtracted_read_pcd";
  string live_topic = "/kinect_both/depth_registered/points";
  string sub_topic = live_topic;
  // string sub_topic = file_topic;
  string viz_topic = "/human/visuals";
  string raw_obs_topic = "/human/observations";
  string pkg_dir = "/home/shray/dev/hydro_ws/src/rim_bmw_experiments/bmw_percep/";
  robo_frame = "base_link";

  //ros
  // ros::init(argc, argv, "sample_tracker");
  // ros::NodeHandle nh;
  // subscribe to input Point Cloud
  pc_sub = nh.subscribe<PointCloudT> 
    (sub_topic, 1, &Tracker3d::pc_call, this);
  // pc_sub_ = nh.subscribe<PCRGB>(sub_topic, 5, &Tracker3d::recvPCCallback, this);

//Publish the human clusters
  db_pc = nh.advertise<pcl::PCLPointCloud2> (pub_topic, 1);
//Publish visualizations for the human
  pub_viz = nh.advertise<visualization_msgs::MarkerArray> (viz_topic, 1);
  pub_raw_obs = nh.advertise<visualization_msgs::MarkerArray> (raw_obs_topic, 1);

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
  PplTrack ppl_tracker(0.25);
  bool ppl_frame_set = false;
  int n_frames=0;
  
  bool done_reading_files = false;
  bool reached = false;
  PointCloudT::Ptr 
    viz_cloud (new PointCloudT);

  while(ros::ok()){
    // //debug
    // cout << "Got to the while? " << endl;
    ros::spinOnce();
    // cout << "Spun" << endl;
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

    //check if cloud empty
    if (cloud->points.size()<1)
      continue;
    //track
    ppl_tracker.estimate(cloud, 
			 clusters,
			 ground_coeffs,
			 robo_loc, got_transform_);

    // set frame-id if first frame
    if(!ppl_frame_set){
      ppl_tracker.set_viz_frame(hum_frame);
      ppl_frame_set=true;}
  
    // ppl_tracker.visualize(pub_viz);
    ppl_tracker.visualize_est(pub_viz);
    ppl_tracker.visualize_obs(pub_raw_obs);
    // pcl::copyPointCloud(*cloud, *viz_cloud);
    if (cloud->points.size()>0){
      pcl::copyPointCloud(*cloud, *tcloud);
      pcl::toPCLPointCloud2(*tcloud, pub_pc);
      pub_pc.header.frame_id = hum_frame;
      db_pc.publish(pub_pc);
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

  new_cloud_available_flag = true;
  
  //also listen to robot--right now
  tf::StampedTransform robo_form;
  try{
    ros::Time a = ros::Time::now()-ros::Duration(0.005);
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
