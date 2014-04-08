#include<opencv2/opencv.hpp>
#include<pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
/**
   Utility functions for OpenCV and PCL
**/

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

namespace cv_utils{

// Changes the color in an image at the specified 8-connected diameter
// to color passed as argument
void paint_at_point(cv::Mat& img, const cv::Point loc, cv::Scalar col, int diam);

// Changes the color in an image at the specified 8-connected diameter 
// to colors in the passed 'orig' image
void paint_at_point(cv::Mat& img, const cv::Point loc, const cv::Mat& orig, int diam);

//Extract only RGB, Depth, and Valid-depth maps from point cloud
bool pc_to_img(const PointCloudT::Ptr& cloud, cv::Mat& d_rgb, cv::Mat& d_depth, 
		 cv::Mat& d_dmask);

// Find blobs in a binary image as a vector of a vector of 2D points
// returns the area of largest blob
  int find_blobs(const cv::Mat &binary, 
			   vector < vector<cv::Point2i> > &blobs);

  // Find blobs in a depth image as a vector of a vector of 2D points
  // returns the area of largest blob
  int find_blobs_depth(const cv::Mat &binary, const cv::Mat &depth,
		       vector < vector<cv::Point2i> > &blobs, float depth_delta);
}
