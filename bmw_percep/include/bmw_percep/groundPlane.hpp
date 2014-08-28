#include <pcl/point_types.h>
#include<pcl/conversions.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include<pcl/common/io.h>
#include <Eigen/SVD>

/**
   
   Class *definition* for estimating the ground/floor plane from RGBD imagery
   Uses PCL.

   -- Uses code from PCL tutorial - http://pointclouds.org/documentation/tutorials/ground_based_rgbd_people_detection.php
**/

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class GroundPlane
{
public:
  //Get plane by clicking points
  GroundPlane(const PointCloudT::Ptr& cloud);
  
  //Get plane from saved file
  GroundPlane (string fileName);

  //Visualize the plane by coloring all the pixels intersecting with ground *pink*
  //TODO: Make it so that I can click a point and get the dot product..
  void visualizePlane(const PointCloudT::Ptr& cloud, double inter_thresh=0.01);

  //Write the ground-coefficients to file
  void writeFile(string fileName);
  
  // creates a mask of the points in an organized point cloud that 
  // lie close to the ground-plane
  //TODO: Eigenize this guy, may be store the distances
  void planePtsMask(const PointCloudT::Ptr& cloud, cv::Mat& mask_im, 
		    double inter_thresh=0.02);

  // project a point-cloud onto the plane and create the image such
  // mask returns the mask of image locations where a point exists
  // mask if contains zeros and ones would be used to select points to project
  void pcProject(const PointCloudT::Ptr& cloud, PointCloudT::Ptr& cloud_projected);

private:
  vector<double> ground_coeffs; // ax+by+cz+d=0

  //direct linear transform on points to find plane using eigen
  bool compute_plane_dlt(const PointCloudT::Ptr& cloud, vector<cv::Point> plane_2d_points);
};
