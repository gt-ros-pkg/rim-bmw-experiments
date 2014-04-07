#include<bmw_percep/pcl_cv_utils.hpp>

/**
   Utility functions for OpenCV and PCL
**/

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;

// Changes the color in an image at the specified 8-connected diameter
// to color passed as argument
void cv_utils::paint_at_point(cv::Mat& img, const cv::Point loc, cv::Scalar col, int diam=1)
{
  int l = max(loc.x-diam, 0);
  int r = min(loc.x+diam, img.cols);
  int u = max(loc.y-diam, 0);
  int d = min(loc.y+diam, img.rows);
  
  for (int i=u; i<d; i++){
    cv::Vec3b* row_i = img.ptr<cv::Vec3b>(i);
    for(int j=l; j<r; j++){
      row_i[j][0] = col[0];
      row_i[j][1] = col[1];
      row_i[j][2] = col[2];
    }
  }
}

// Changes the color in an image at the specified 8-connected diameter 
// to colors in the passed 'orig' image
void cv_utils::paint_at_point(cv::Mat& img, const cv::Point loc, 
			      const cv::Mat& orig, int diam=1)
{
  int l = max(loc.x-diam, 0);
  int r = min(loc.x+diam, img.cols);
  int u = max(loc.y-diam, 0);
  int d = min(loc.y+diam, img.rows);
  
  for (int i=u; i<d; i++){
    cv::Vec3b* row_i = img.ptr<cv::Vec3b>(i);
    const cv::Vec3b* orig_i = orig.ptr<cv::Vec3b>(i);
    for(int j=l; j<r; j++){
      row_i[j][0] = orig_i[j][0];
      row_i[j][1] = orig_i[j][1];
      row_i[j][2] = orig_i[j][2];
    }
  }
}

//Extract RGB, Depth, and Valid-depth maps from point cloud
bool cv_utils::pc_to_img(const PointCloudT::Ptr& cloud, cv::Mat& d_rgb, 
			 cv::Mat& d_depth, cv::Mat& d_dmask)
{
  int pc_rows=cloud->height;
  int pc_cols=cloud->width;
  
  //check if conversion possible
  if (cloud->isOrganized()){

    //TODO: Check if allocated Mat can be created
    if (!cloud->empty()){
      // allocate if neccessary
      if (d_rgb.size()!=cv::Size(pc_rows, pc_cols) || d_rgb.depth()!= CV_8UC3)
	d_rgb.create(pc_rows, pc_cols, CV_8UC3);

      if (d_depth.size()!=cv::Size(pc_rows, pc_cols) || d_depth.depth()!= CV_64F)
	d_depth = cv::Mat::zeros(pc_rows, pc_cols, CV_64F);
      else
	d_depth = cv::Scalar(0.0);

      if (d_dmask.size()!=cv::Size(pc_rows, pc_cols) || d_dmask.depth()!= CV_8U)
	d_dmask = cv::Mat::zeros(pc_rows, pc_cols, CV_8U);
      else
	d_dmask = cv::Scalar(0);

      for (int r=0; r<pc_rows; r++){

	cv::Vec3b* rgb_r = d_rgb.ptr<cv::Vec3b> (r);
	double* depth_r = d_depth.ptr<double> (r);
	unsigned char* dmask_r = d_dmask.ptr<unsigned char> (r);

	for (int c=0; c<pc_cols; c++){
	  PointT point = cloud->at(c,r);
	  //set depth and depth-mask if not NaN
	  if (!isnan(point.z)){
	    depth_r[c] = (double)point.z;
	    dmask_r[c] = (unsigned char)255;
	  }
	  
	  //set colors
	  Eigen::Vector3i rgb = point.getRGBVector3i();
	  rgb_r[c][0] = rgb[2];
	  rgb_r[c][1] = rgb[1];
	  rgb_r[c][2] = rgb[0];
	}
      }
    }
    else{
      cout << "\n Cloud Empty!" << endl;
      return false;
    }
  }
  else{
    cout << endl << "Cloud Unorganized.." << endl;
    return false;
  }

  return true;
}
