#include <stdio.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <sensor_msgs/PointCloud2.h>

using namespace std;

typedef pcl::PointXYZ PXYZ;
typedef pcl::PointCloud<PXYZ> PCXYZ;

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

class BackgroundSubtract
{
public:
  BackgroundSubtract(ros::NodeHandle& nh)
    : nh_(nh)
  {
    pc_sub_ = nh.subscribe<PCRGB>("pc_in", 1, &BackgroundSubtract::recvPCCallback, this);
    pc_pub_ = nh.advertise<PCRGB>("pc_out", 1);

    //front kinect or back
    cout << "Front(f) or Back(b) Kinect?" << endl;
    char kin; cin >> kin;
    string kinect_or;

    if(kin=='f')
      kinect_or = "front";
    else if(kin=='b')
      kinect_or = "back";
    else{
      cout << "Type f or b."<<endl;
      cout << "No background is being subtracted!" << endl;
      return;
    }      
    
    //Read in PC-max and min
    string min_file = "data/" + kinect_or + "/min.pcd";
    string max_file = "data/" + kinect_or + "/max.pcd";

    if(pcl::io::loadPCDFile<PXYZ> (min_file, pc_min_) == -1 ){
      cout << "Min File not found. Path = " << min_file << endl;  
    }
    if(pcl::io::loadPCDFile<PXYZ> (max_file, pc_max_) == -1 ){
      cout << "Max File not found. Path = " << max_file << endl;  
    }
  }
protected:
  void recvPCCallback(const PCRGB::ConstPtr& msg);
  void bg_sub();

  PCXYZ pc_min_, pc_max_; // Range of values that belong to the background
  PCRGB pc_in_;
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;
};

void BackgroundSubtract::recvPCCallback(const PCRGB::ConstPtr& msg)
{
  //copy to private var
  pc_in_ = *msg;

  pcl::PointIndices inds;
  
  //background subtract
  bg_sub(inds);

  //remove robot

  //publish subbed point cloud
  pc_pub_.publish(pc_in_);
}

void BackgroundSubtract::bg_sub(pcl::PointIndices &inds)
{

  //TODO: clip too far points 
  for(size_t i=0; i<pc_in_.size(); i++){
    PXYZ p_min = pc_min_[i];
    PXYZ p_max = pc_max_[i];
    PRGB pt = pc_in_.at(i);
    bool push=false;

    if(!isnan(pt.z)){
      if(!isnan(p_min.z)){
	if(pt.z>p_min.z && pt.z<p_max.z){
	  inds.indices.push_back(i);
	}
      }
    }

  }
  
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "bg_subtract");
    ros::NodeHandle nh_priv("~");
    
    BackgroundSubtract bg_sub(nh_priv);
    ros::spin();
    return 0;
}
