#include <pcl/point_types.h>
#include<pcl/conversions.h>
//#include <pcl/sample_consensus/sac_model_plane.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>

/**
   
   Class *definition* for estimating the ground/floor plane from RGBD imagery
   Uses PCL.

   -- Uses code from PCL tutorial - http://pointclouds.org/documentation/tutorials/ground_based_rgbd_people_detection.php
**/

using namespace std;

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class GroundPlane
{
public:
  //Get plane by clicking points
  GroundPlane(const PointCloudT::Ptr& cloud);
  
  //Get plane from saved file
  GroundPlane (string fileName);

  //Visualize the plane by coloring all the pixels intersecting with ground *pink*
  void visualizePlane(const PointCloudT::Ptr& cloud, double inter_thresh=0.01);

  //Write the ground-coefficients to file
  void writeFile(string fileName);

private:
  vector<double> ground_coeffs; // ax+by+cz+d=0

  //direct linear transform on points to find plane
  bool compute_plane_dlt(const PointCloudT::Ptr& cloud, vector<cv::Point> plane_2d_points);

};
