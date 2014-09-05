#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common_headers.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/people/person_cluster.h>
#include <limits>
/** 
    Utility functions for People Detection
**/

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PointCloudX;

using namespace std;

namespace ppl_detection{

  void find_euclid_blobs(PointCloudT::ConstPtr cloud, 
			 PointCloudT::Ptr viz_cloud, 
			 vector<cv::Point3f> clusters, 
			 int& max_blob_id,
			 const Eigen::VectorXf ground_coeffs,
			 float leaf_size=0.01);

  bool get_min_ground_dist (const PointCloudT::Ptr cloud, 
			    pcl::people::PersonCluster<PointT> 
			    person_c, 
			    const Eigen::VectorXf ground_coeffs, 
			    double sqrt_ground_coeffs, double min_dist);

  void mergeClustersCloseInFloorCoordinates 
  ( const PointCloudT::Ptr cloud, 
    std::vector<pcl::people::PersonCluster<PointT> >& input_clusters,
    std::vector<pcl::people::PersonCluster<PointT> >& output_clusters, 
    const Eigen::VectorXf ground_coeffs_, double sqrt_ground_coeffs_);

  void find_ppl_clusters
  (const PointCloudT::Ptr cloud, 
   vector<pcl::PointIndices>& init_indices, 
   std::vector<pcl::people::PersonCluster<PointT> >& clusters,
   const Eigen::VectorXf ground_coeffs,
   const int max_c_size,
   const int min_c_size);

void rm_ppl_clusters
( const PointCloudT::Ptr cloud, 
  std::vector<pcl::people::PersonCluster<PointT> >& in_cl,
  std::vector<pcl::people::PersonCluster<PointT> >& out_cl, 
  const Eigen::Vector4f ground_coeffs_, double sqrt_ground_coeffs_,
  const float max_ht, const float min_ht, 
  const float max_gr_dist,
  const int max_c_size,
  const int min_c_size);
}
