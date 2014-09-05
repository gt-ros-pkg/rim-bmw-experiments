#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/io.h>
#include <Eigen/SVD>

#include<ros/ros.h>

/**
   
   Class *definition* for tracking people from RGBD imagery.
   Uses PCL.
**/

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef Eigen::Vector3f ClusterPoint;

class PplTrack
{
public:
  //Constructor
  PplTrack(Eigen::Vector4f ground_coeffs);
  
  void estimate(vector<vector<ClusterPoint> > clusters);
  
  
  void visualize(ros::Publisher pub);

private:
  vector<int> person_ids_;
  PointCloudT::Ptr cur_cloud_;
  Eigen::Vector4f ground_coeffs_;
  string visual_topic_;
  ClusterPoint cur_pos;
  void estimate(vector<ClusterPoint> cluster);
  int getOneCluster(const vector<vector<ClusterPoint> > clusters);
};
