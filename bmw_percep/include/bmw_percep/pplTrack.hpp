#ifndef PPL_TRACK
#define PPL_TRACK

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
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common_headers.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <bmw_percep/ppl_detection.hpp>
#include <bmw_percep/shr_cv_utils.hpp>

//ros-includes
#include<ros/ros.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<geometry_msgs/PoseStamped.h>

/**
   
   Class *definition* for tracking people from RGBD imagery.
   Uses PCL.
**/

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef Eigen::Vector3f ClusterPoint;
struct ClusterStats{
ClusterPoint mean;
ClusterPoint std;
};
typedef struct ClusterStats ClusterStats;

class PplTrack
{
public:
  //Constructor if ground plane required
  PplTrack(Eigen::Vector4f ground_coeffs);
  
  //If pointcloud in table_link frame
  PplTrack(float z);

  void estimate(vector<vector<ClusterPoint> > clusters);
  
  // Takes in point cloud 
  // (background subtracted or not undecided)
  // Performs the complete pipeline of PC 
  // processing and then estimates the position of
  // person or people
  void estimate(PointCloudT::Ptr& cloud, 
		vector<vector<ClusterPoint> > &clusters,
		const Eigen::VectorXf ground_coeffs,
		const Eigen::Vector3f robo_loc,
		float leaf_size=0.06);
  
  void visualize(ros::Publisher pub);

  void set_viz_frame(string viz_frame)
  {viz_frame_ = viz_frame;}

  //removes points from the cloud that are not part of 
  //the defined workspace..
  void workspace_limit(PointCloudT::Ptr& cloud);

  //remove points belonging to the robot
  //currently removes an infinite .5m cylinder
  //from the base_link
  void robot_remove(PointCloudT::Ptr &cloud,
		    Eigen::Vector3f robo_loc);

private:
  bool table_link;
  vector<int> person_ids_;
  PointCloudT::Ptr cur_cloud_;
  Eigen::Vector4f ground_coeffs_;
  string viz_frame_;
  vector<ClusterPoint> cur_pos_;
float max_height_, min_height_, max_dist_gr_;
int max_c_size_, min_c_size_;

//member functions
  void estimate(vector<ClusterPoint> cluster);
  int getOneCluster(const vector<vector<ClusterPoint> > clusters);
void merge_floor_clusters(PointCloudT::Ptr cloud, 
			    vector<pcl::PointIndices> cluster_indices);
};

#endif
