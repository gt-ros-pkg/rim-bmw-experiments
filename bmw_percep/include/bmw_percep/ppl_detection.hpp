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
}
