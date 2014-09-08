#include <iostream>
#include <pcl/point_cloud.h>
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


/**
   Subscribe to two point clouds, transform into 
   one frame. Merge them and then publish as a
   new Point Cloud.
**/

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudSM;

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
//GLOBALs
//TODO: Get Rid of 'em
string front_frame, back_frame;
PointCloudSM::Ptr front_pc;
PointCloudSM::Ptr back_pc;
bool new_pc_f, new_pc_b;

//TODO: Time stamp checks on the two pointclouds
int main(int argc, char** argv)
{
  //initialize shared pointers
  front_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  back_pc = boost::shared_ptr<PointCloudSM>(new PointCloudSM);
  
  //ros
  ros::init(argc, argv, "merge_pc");
  ros::NodeHandle nh;
  ros::Publisher pc_pub = nh.advertise<PointCloudSM> 
    ("/kinect_both/depth_registered/points", 1);
  ros::Subscriber front_sub = nh.subscribe<PointCloudSM> 
    ("/kinect_front/depth_registered/points", 1, front_call);
  ros::Subscriber back_sub = nh.subscribe<PointCloudSM> 
    ("/kinect_back/depth_registered/points", 1, back_call);

  //world frame for easy clipping of workspace
  string new_frame = "table_link";

  //Point Clouds
  PointCloudSM::Ptr pub_pc(new PointCloudSM);
  PointCloudSM::Ptr new_b_pc(new PointCloudSM), new_f_pc(new PointCloudSM);

  //Listeners for both transformations
  tf::TransformListener trans_back_table;
  tf::TransformListener trans_front_table;

  //Transformers for the PointClouds
  pclTransform back_transform, front_transform;

  //listen for transform until one is gotten
  //since its static, don't look for transform afterwards
  bool found_back_t=false, found_front_t=false;

  new_pc_f = false;
  new_pc_b = false;

  while (nh.ok() && !(found_back_t && found_front_t)){
    ros::spinOnce();
    
    if (!(new_pc_f && new_pc_b))
      continue; // spin again if both Pointclouds not get

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

  new_pc_f = false;
  new_pc_b = false;
  //pcl::transformPointCloud(front_pc, pub_pc, TransMat ); 

  //debug
  // cout << "\nBoth transformed, time to merge!\n";
  
  boost::timer timer_transform, timer_concat, timer_total;
  double time_transform=0.0, time_concat=0.0, time_total=0.0;
  unsigned long n_frames=0;

  while(ros::ok()){

    timer_total.restart();

    ros::spinOnce();

    //if both PCs new then concatenate
    if (new_pc_f && new_pc_b){
      
      timer_transform.restart();

      // pcl::transformPointCloud(*back_pc, *new_b_pc, back_transform.translation, 
      // 			       back_transform.rotation);
      // pcl::transformPointCloud(*front_pc, *pub_pc, front_transform.translation, 
      // 			       front_transform.rotation);

      // cout << "Get to voxelize??" << endl;
      // //Voxelize-first
      // voxelize_cloud(back_pc, new_b_pc);
      // voxelize_cloud(front_pc, new_f_pc);

      // pcl::transformPointCloud(*new_b_pc, *back_pc, back_transform.translation, 
      // 			       back_transform.rotation);
      // pcl::transformPointCloud(*new_f_pc, *pub_pc, front_transform.translation, 
      // 			       front_transform.rotation);

      pcl::transformPointCloud(*back_pc, *new_b_pc, back_transform.translation, 
      			       back_transform.rotation);
      pcl::transformPointCloud(*front_pc, *pub_pc, front_transform.translation, 
      			       front_transform.rotation);


      time_transform+=timer_transform.elapsed();

      //concatenate
      // pcl::transformPointCloud(front_pc, pub_pc, TransMat ); 
      timer_concat.restart();
      *pub_pc += *new_b_pc;
      time_concat += timer_concat.elapsed();

      //set frame
      pub_pc->header.frame_id = new_frame;

      new_pc_f = false;
      new_pc_b = false;

      //publish
      pc_pub.publish(pub_pc);

      time_total+=timer_total.elapsed();
      n_frames++;

      //debug
      // if (n_frames%10 == 0)
      // 	{
      // 	  cout << "\n**********COMPUTE-TIMES**********\n";
      // 	  cout << "Transform = " << time_transform/n_frames << " seconds\n";
      // 	  cout << "Concatenation = " << time_concat/n_frames << " seconds\n";
      // 	  cout << "Total = " << time_total/n_frames << " seconds" << endl;

      // 	}
	
    }

  }

  return 0;
}

// callback:
void front_call(const PointCloudSM::ConstPtr& cloud)
{
  if (!new_pc_f){
    front_frame = cloud->header.frame_id;
    *front_pc = *cloud;
    new_pc_f = true;
  }
    
}

void back_call(const PointCloudSM::ConstPtr& cloud)
{
  if (!new_pc_b){
    back_frame = cloud->header.frame_id;
    *back_pc = *cloud;
    new_pc_b = true;
  }
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
