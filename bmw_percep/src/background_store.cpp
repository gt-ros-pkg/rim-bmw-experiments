#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <sensor_msgs/PointCloud2.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#define BG_FRAMES 30

using namespace std;
using namespace boost::accumulators;

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

typedef pcl::PointXYZ PXYZ;
typedef pcl::PointCloud<PXYZ> PCXYZ;

class BackgroundStore
{
public:
  BackgroundStore(ros::NodeHandle& nh)
    : nh_(nh)
  {
    pc_sub_ = nh.subscribe<PCRGB>("/kinect_front/depth_registered/points", 
				  1, &BackgroundStore::recvPCCallback, this);
    n_frames_ = 0;
    
  }
protected:
  void recvPCCallback(const PCRGB::ConstPtr& msg);
  void gen_bg(); //Store the generated stats as bg-images
  void take_bg_stats(); //Keeps statistics on the point clouds
  vector<accumulator_set< float, stats<tag::mean, tag::variance> > > acc_vec;

  PCXYZ pc_min_, pc_max_; // Range of values that belong to the background
  PCRGB pc_in_;
  size_t n_frames_;
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
};

void BackgroundStore::recvPCCallback(const PCRGB::ConstPtr& msg)
{

  if (n_frames_>BG_FRAMES){
    cout << "Done with frames. End it." << endl;
    return;
  }
  else{
    pc_in_ = *msg;
    take_bg_stats();
  }
}

void BackgroundStore::gen_bg()
{

  pcl::copyPointCloud(pc_in_, pc_max_);
  pcl::copyPointCloud(pc_in_, pc_min_);  

  for(size_t i=0; i<acc_vec.size(); i++){
    float pmean = boost::accumulators::mean(acc_vec[i]);
    float pstd = pow(boost::accumulators::variance(acc_vec[i]), 1./2.);
    pc_max_[i].z = pmean+2*pstd;
    pc_min_[i].z = pmean-2*pstd;

  }

  system("mkdir data/backg-temp");
  string path="data/backg-temp/";

  //store as PCD

  pcl::io::savePCDFile(path+"min.pcd", pc_min_);
  pcl::io::savePCDFile(path+"max.pcd", pc_max_);
}

void BackgroundStore::take_bg_stats()
{
  if (n_frames_==0) //first frame
    acc_vec.resize(pc_in_.size());

  cout << "PC Size = " << pc_in_.size() << endl;

  for(size_t i=0; i< pc_in_.size(); ++i){
    PRGB pt = pc_in_[i];
    if(!isnan(pt.z))
      acc_vec[i](pt.z);
    }
  
  n_frames_++;
  cout << "Frame No. = " << n_frames_ << endl;
  if (n_frames_>BG_FRAMES){
    gen_bg();
  }

}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "bg_store");
    ros::NodeHandle nh_priv("~");
    BackgroundStore bg_store(nh_priv);
    ros::spin();
    return 0;
}
