#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/registration/transforms.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <boost/timer.hpp>
#include <pcl/filters/voxel_grid.h>
#include<bmw_percep/shr_cv_utils.hpp>
//#include<bmw_percep/groundPlane.hpp>

//ros-includes
#include<ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
/**
   Subscribe to two point clouds, transform into 
   one frame. Merge them and then publish as a
   new Point Cloud.
**/

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;
typedef sensor_msgs::PointCloud2 PCMsg;

struct pclTransform{
  Eigen::Vector3d translation;
  Eigen::Quaterniond rotation;
};

// typedef sensor_msgs::PointCloud PointCloudSM;

using namespace std;

// callback:
void front_call(const PointCloudSM::ConstPtr&);
void back_call(const PointCloudSM::ConstPtr&);
void voxelize_cloud(PointCloudSM::Ptr cloud, 
		    PointCloudSM::Ptr filtered_cloud, 
		    float leaf_size=0.06);
void callback(const PCMsg::ConstPtr& front_pc, 
	      const PCMsg::ConstPtr& back_pc );

void get_transform_fb( string back_frame, string front_frame, 
		       pclTransform &back_transform, pclTransform &front_transform);

//GLOBALs
//TODO: Get Rid of 'em
//Transformers for the PointClouds
pclTransform back_transform, front_transform;
bool first_frame;
string front_frame, back_frame;
bool new_pc_f, new_pc_b;
ros::Publisher pc_pub;
string new_frame;
PointCloudSM::Ptr back_pc, new_b_pc, pub_pc, front_pc;

//TODO: Time stamp checks on the two pointclouds
int main(int argc, char** argv)
{
  first_frame=true;
  //initialize shared pointers
  front_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  back_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  new_b_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  pub_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM);


  //ros
  
  string bg_back_topic = "background_sub_back/pc1_out";
  string bg_front_topic = "background_sub_front/pc1_out";
  
  ros::init(argc, argv, "merge_pc");
  ros::NodeHandle nh;
  pc_pub = nh.advertise<PointCloudSM> 
    ("/kinect_both/depth_registered/points", 1);
  // ros::Subscriber front_sub = nh.subscribe<PointCloudSM> 
  //   ("/kinect_front/depth_registered/points", 1, front_call);
  // ros::Subscriber back_sub = nh.subscribe<PointCloudSM> 
  //   ("/kinect_back/depth_registered/points", 1, back_call);

message_filters::Subscriber<PCMsg> 
  front_sub(nh, bg_front_topic, 1);
message_filters::Subscriber<PCMsg> 
    back_sub(nh, bg_back_topic, 1);

typedef message_filters::sync_policies::ApproximateTime<PCMsg, PCMsg> MySyncPolicy;

message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(4), back_sub, front_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2));

  //world frame for easy clipping of workspace
  new_frame = "table_link";

  //Point Clouds
  PointCloudSM::Ptr pub_pc(new PointCloudSM);
  PointCloudSM::Ptr new_b_pc(new PointCloudSM), new_f_pc(new PointCloudSM);


  boost::timer timer_transform, timer_concat, timer_total;
  double time_transform=0.0, time_concat=0.0, time_total=0.0;
  unsigned long n_frames=0;


  ros::spin();

  return 0;
}

void callback(const PCMsg::ConstPtr& front_pc_, const PCMsg::ConstPtr& back_pc_ )
{
  
  pcl::PCLPointCloud2 back_pc1;

  pcl_conversions::toPCL(*back_pc_, back_pc1);
  pcl::fromPCLPointCloud2(back_pc1, *back_pc);

  pcl::PCLPointCloud2 front_pc1;
  pcl_conversions::toPCL(*front_pc_, front_pc1);
  pcl::fromPCLPointCloud2(front_pc1, *front_pc);

  back_frame = front_pc->header.frame_id;
 
  front_frame = back_pc->header.frame_id;
  

  if(first_frame){
    get_transform_fb(back_frame, front_frame, back_transform, front_transform);
    first_frame=false;
  }

  //transform points
  pcl::transformPointCloud(*back_pc, *new_b_pc, front_transform.translation, 
      			       front_transform.rotation);
  pcl::transformPointCloud(*front_pc, *pub_pc, back_transform.translation, 
  			   back_transform.rotation);


  *pub_pc += *new_b_pc;
  
  pub_pc->header.frame_id = new_frame;
  pc_pub.publish(*pub_pc);

}

 void voxelize_cloud(PointCloudSM::Ptr cloud, 
		     PointCloudSM::Ptr filtered_cloud, 
		    float leaf_size/*=0.01*/)
{
  float voxel_size=leaf_size;

  //Voxelize the space
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  //PointCloudSM::Ptr cloud_filtered(new PointCloudSM);
  vg.setInputCloud(cloud);
  vg.setLeafSize(voxel_size, voxel_size, voxel_size);
  vg.filter(*filtered_cloud);
    
  // //debug
  // cout << "PointCloud after filtering has: " << 
  //   filtered_cloud->points.size ()  << " data points." << endl;
  // cout << "Vs. " << cloud->points.size() << " before." << endl;

}

void get_transform_fb( string back_frame, string front_frame, 
		       pclTransform &back_transform, pclTransform &front_transform)
{
  //Listeners for both transformations
  tf::TransformListener trans_back_table;
  tf::TransformListener trans_front_table;


  //listen for transform until one is gotten
  //since its static, don't look for transform afterwards
  bool found_back_t=false, found_front_t=false;

  while (!(found_back_t && found_front_t)){
    
    //Back kinect transform
    if(!found_back_t){
      tf::StampedTransform t_back;
      try{
	trans_back_table.waitForTransform(new_frame, back_frame, ros::Time(0),
					  ros::Duration(10.0));
	trans_back_table.lookupTransform(new_frame, back_frame, ros::Time(0),
					 t_back);
      }
      catch(tf::TransformException &ex){
	cout << ex.what() << endl;
	ROS_ERROR("%s", ex.what());
	continue;
      }
      //Store transform
      found_back_t=true;
      tf::vectorTFToEigen(t_back.getOrigin(), back_transform.translation);      
      tf::quaternionTFToEigen(t_back.getRotation(), back_transform.rotation);
      
    }

    //Front kinect transform
    if(!found_front_t){
      tf::StampedTransform t_front;
      try{
	trans_front_table.waitForTransform(new_frame, front_frame, ros::Time(0),
					  ros::Duration(10.0));
	trans_front_table.lookupTransform(new_frame, front_frame, ros::Time(0),
					 t_front);
      }
      catch(tf::TransformException &ex){
	cout << ex.what() << endl;
	ROS_ERROR("%s", ex.what());
	continue;
      }
      //Store transform
      found_front_t=true;
      tf::vectorTFToEigen(t_front.getOrigin(), front_transform.translation);      
      tf::quaternionTFToEigen(t_front.getRotation(), front_transform.rotation);
      
      // pcl::transformPointCloud(front_pc, new_b_pc, front_transform.translation, 
      // 			       front_transform.rotation);
    }

    // pub_pc = new_b_pc + new_f_pc;
    // pcl::concatenatePointCloud(new_b_pc, new_f_pc, pub_pc);
    
  }

  //pcl::transformPointCloud(front_pc, pub_pc, TransMat ); 

  //debug
  // cout << "\nBoth transformed, time to merge!\n";
  
  //Convert into homogenous transformation matrices
  Eigen::Matrix4f back_trans_mat;
  shr_cv_utils::to_trans_mat(back_transform.rotation, back_transform.translation,
			     back_trans_mat);
  Eigen::Matrix4f front_trans_mat;
  shr_cv_utils::to_trans_mat(front_transform.rotation, front_transform.translation,
			     front_trans_mat);

  return;

}
