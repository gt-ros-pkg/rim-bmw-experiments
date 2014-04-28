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
   Utility functions for OpenCV and PCL
**/

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PointCloudX;

using namespace std;

namespace cv_utils{

  // Changes the color in an image at the specified 8-connected diameter
  // to color passed as argument
  void paint_at_point(cv::Mat& img, const cv::Point loc, cv::Scalar col, 
		      int diam);

  // Changes the color in an image at the specified 8-connected diameter 
  // to colors in the passed 'orig' image
  void paint_at_point(cv::Mat& img, const cv::Point loc, 
		      const cv::Mat& orig, int diam);

  //Extract only RGB, Depth, and Valid-depth maps from point cloud
  bool pc_to_img(const PointCloudT::Ptr& cloud, cv::Mat& d_rgb, 
		 cv::Mat& d_depth, cv::Mat& d_dmask);

  //Extract only RGB, Depth, and Valid-depth maps from point cloud
  bool pc_to_img(PointCloudT::ConstPtr& cloud, cv::Mat& d_rgb, 
		 cv::Mat& d_depth, cv::Mat& d_dmask);


  bool pc_to_depth(const PointCloudX::Ptr& cloud, 
		   cv::Mat& d_depth, cv::Mat& d_dmask);

  // Find blobs in a binary image, returns a vector of a vector of 2D
  // points
  int find_blobs(const cv::Mat &binary, 
		 vector < vector<cv::Point2i> > &blobs);

  // Find blobs in a binary image as a vector of a vector of 2D points
  // returns the ID of largest blob
  int find_blobs(const cv::Mat &binary, 
		 vector < vector<cv::Point2i> > &blobs,
		 int& max_blob_id);

  // Find blobs in a depth image as a vector of a vector of 2D points
  // returns the area of largest blob
  int find_blobs_depth(const cv::Mat &binary, const cv::Mat &depth,
		       vector < vector<cv::Point2i> > &blobs, float depth_delta);

  // Finds blobs from an organized 3D point cloud and a binary
  // mask over the image of the cloud
  // returns the clusters as 3D points
  // Used pcl tutorial on euclidean clustering as reference
  void find_euclid_blobs(PointCloudX::ConstPtr cloud, 
			 pcl::PointCloud<pcl::PointXYZRGB>::Ptr viz_cloud, 
			 const cv::Mat &mask, 
			 vector<cv::Point3f> clusters, int& max_blob_id,
			 float leaf_size=0.01);

  void find_euclid_blobs(PointCloudX::ConstPtr cloud, 
			 pcl::PointCloud<pcl::PointXYZ>::Ptr viz_cloud, 
			 const cv::Mat &mask, 
			 vector<cv::Point3f> clusters, int& max_blob_id,
			 float leaf_size=0.01);
  
  void find_euclid_blobs(PointCloudT::ConstPtr cloud, 
			 PointCloudT::Ptr viz_cloud, 
			 vector<cv::Point3f> clusters, int& max_blob_id,
			 const Eigen::VectorXf ground_coeffs,
			 float leaf_size=0.01);
  
void find_ppl_clusters(const PointCloudT::Ptr cloud, 
		  vector<pcl::PointIndices>& init_indices, 
		  std::vector<pcl::people::PersonCluster<PointT> >& clusters,
		  const Eigen::VectorXf ground_coeffs);

void mergeClustersCloseInFloorCoordinates 
(const PointCloudT::Ptr cloud, 
 std::vector<pcl::people::PersonCluster<PointT> >& input_clusters,
 std::vector<pcl::people::PersonCluster<PointT> >& output_clusters, 
 const Eigen::VectorXf ground_coeffs_, double sqrt_ground_coeffs_);
 
  bool get_min_ground_dist(const PointCloudT::Ptr cloud, 
			   pcl::people::PersonCluster<PointT> person_c, 
			   const Eigen::VectorXf ground_coeffs, 
			   double sqrt_ground_coeffs, double min_dist);

  void depth_bgSub(PointCloudT::ConstPtr cloud, PointCloudT::Ptr bgCloud, 
		   const cv::Mat& bg);

  void depth_bgSub( PointCloudT::ConstPtr cloud, PointCloudT::Ptr bgCloud, 
		   cv::BackgroundSubtractorMOG2 cvBg);

  void find_euclid_blobs(PointCloudT::ConstPtr cloud, 
			 PointCloudT::Ptr viz_cloud, 
			 vector<cv::Point3f> clusters, int& max_blob_id,
			 const Eigen::VectorXf ground_coeffs, cv::Mat bg,
			 float leaf_size=0.01);

  void find_euclid_blobs(PointCloudT::ConstPtr cloud, 
			 PointCloudT::Ptr viz_cloud, 
			 vector<cv::Point3f> clusters, int& max_blob_id,
			 const Eigen::VectorXf ground_coeffs, 
			 cv::BackgroundSubtractorMOG2 cvBg,
			 float leaf_size=0.01);

  bool pc_to_img_no_filter( const PointCloudT::Ptr& cloud, cv::Mat& d_rgb, 
			    cv::Mat& d_depth, cv::Mat& d_dmask);
  bool pc_to_img_no_filter( PointCloudT::ConstPtr& cloud, cv::Mat& d_rgb, 
			    cv::Mat& d_depth, cv::Mat& d_dmask);
}
