#ifndef PPL_TRACK
#define PPL_TRACK

#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <opencv2/opencv.hpp>

#include <fstream>
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
#include <pcl/filters/random_sample.h>
#include <bmw_percep/ppl_detection.hpp>
#include <bmw_percep/shr_cv_utils.hpp>
//#include <bmw_percep/particleFilter.hpp>
#include <bmw_percep/kalmanFilterAcc.hpp>
#include <queue>
#include <math.h>
#include <opencv2/opencv.hpp>

//ros-includes
#include<ros/ros.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<geometry_msgs/PoseStamped.h>

#include <boost/lexical_cast.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/median.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/timer.hpp>

#define RANDOM_COLORS false

/**
   
   Class *definition* for tracking people from RGBD imagery.
   Uses PCL.

**/

using namespace std;
using namespace boost::accumulators;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef Eigen::Vector3f ClusterPoint;
struct ClusterStats{
ClusterPoint mean;
ClusterPoint var; //variance
ClusterPoint min; 
ClusterPoint max; //all these stats are taken independent in the dimensions
ClusterPoint median;
};
typedef struct ClusterStats ClusterStats;
typedef Eigen::Matrix<int, 2, 1> Index2D;

//element of the height map
struct HMapEl{
  float height; // max height
  Eigen::Vector2f loc;
  vector<Eigen::Vector2f> p_ind; // point cloud indices
  bool in_h_range;
};
typedef struct HMapEl HMapEl;

struct PersProp{ //properties of the person
Eigen::Vector2f pos;
Eigen::Vector2f vel;
  Eigen::Vector2f inn_cyl;
  Eigen::Vector2f out_cyl;
  float height;
};

typedef struct PersProp PersProp;

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
		const Eigen::Vector3f robo_loc,
	      bool got_tf_robot, ros::Time pc_time,
		float leaf_size=0.06);
  
  // void visualize(ros::Publisher pub);

  void visualize(ros::Publisher pub, Eigen::Vector3f color, PersProp person,
		 string name_space);

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
//Apply rules to "rule-out" the clusters not
//satisfying human rules
void rm_clusters_rules(const PointCloudT::Ptr &cloud,
			 vector<pcl::PointIndices> &cs_indices);


//assign clusters to the private member
void assign_ppl_clusters(PointCloudT::ConstPtr cloud, 
			   const vector<pcl::PointIndices> cluster_indices);

//Get statistics on the clusters
void get_clusters_stats(PointCloudT::ConstPtr cloud, 
			  const vector<pcl::PointIndices> cluster_indices,
			  vector<ClusterStats>& clusters_stats);
  //Visualize the detected human position / observation for filter
  void visualize_obs(ros::Publisher pub)
  {visualize(pub, Eigen::Vector3f(0.0, 1.0, 0.0), pers_obs_, "human/observations");}

  void visualize_est(ros::Publisher pub)
  {visualize(pub, Eigen::Vector3f(1.0, 0.0, 0.0), pers_est_, "human/visuals");}

  //Publishes the position of the observation, position and velocity of the
  //filtered belief of the human
  void pub_obs_est(ros::Publisher pub_o, ros::Publisher pub_e_p, 
		   ros::Publisher pub_e_v );

private:
bool table_link;
int person_id_; // the index of the person from the person clusters
PointCloudT::Ptr cur_cloud_;
Eigen::Vector4f ground_coeffs_;
string viz_frame_;
vector<vector<ClusterPoint> > per_cs_; // person clusters
vector<ClusterStats> per_stats_;
queue<PersProp> history_per_stats_; //keeps history of person stats
int history_size_; // number of frames to keep in history
vector<ClusterPoint> cur_pos_;
float max_height_, min_height_, max_dist_gr_;
int max_c_size_, min_c_size_;
bool more_than_one_;
ros::Time pub_time_; //time-stamp associated with human estimation
  ros::Time prev_time_;
  float jerk_std_;
  KalmanFilterAcc kf_tracker_;
bool currently_filtering_; //does the particle filter need reinitialization?
  PersProp pers_obs_, pers_est_; //current person observation and the estimate
  Eigen::Vector3f ws_max_, ws_min_; //max and min range for workspace
  int file_no_;

//member functions
  void cluster_head(vector<vector<HMapEl> > hmap, size_t x, size_t y, 
		    int h_x_span, int h_y_span, 
		    vector<Index2D> &i_list);
  void get_head_center(int c_ind, Eigen::Vector2f &h_center);

  // //find 2D center for the head
  // void find_head_center(int c_ind);

  // //create a 2D map for heights in a cluster
  // void create_height_map(int c_ind, vector<HMapEl> &hmap);

  void write_clusters_disk();
void reset_vals(); //resets the variables for calculating anew
void estimate(vector<ClusterPoint> cluster);
int getOneCluster(const vector<vector<ClusterPoint> > clusters);
void merge_floor_clusters(const PointCloudT::Ptr cloud, 
			    vector<pcl::PointIndices> &cluster_indices);
void clear_history();
  void set_observation(); //set observation from the clusters
  void set_estimate(); // set size parameters from observation 

};

#endif
