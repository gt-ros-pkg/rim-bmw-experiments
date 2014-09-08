#include<bmw_percep/shr_cv_utils.hpp>

/**
   Utility functions for OpenCV and PCL
**/

// typedef pcl::PointXYZRGB PointT;
// typedef pcl::PointCloud<PointT> PointCloudT;

// typedef pcl::PointXYZ PointX;
// typedef pcl::PointCloud<PointX> PointCloudX;

using namespace std;

//Extract RGB, Depth, and Valid-depth maps from point cloud
bool shr_cv_utils::pc_to_img(const PointCloudT::Ptr& cloud, cv::Mat& d_rgb, 
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
	d_depth = cv::Scalar(10.0);

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
	  // if (!isnan(point.z) && point.z<4.0 && point.z>0.6){
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

//Extract RGB, Depth, and Valid-depth maps from point cloud
bool shr_cv_utils::pc_to_img( PointCloudT::ConstPtr& cloud, cv::Mat& d_rgb, 
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
	d_depth = cv::Scalar(10.0);

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
	  // if (!isnan(point.z) && point.z<4.0 && point.z>0.6){
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

bool shr_cv_utils::pc_to_depth(const PointCloudX::Ptr& cloud, 
			 cv::Mat& d_depth, cv::Mat& d_dmask)
{
  int pc_rows=cloud->height;
  int pc_cols=cloud->width;
  
  //check if conversion possible
  if (cloud->isOrganized()){

    //TODO: Check if allocated Mat can be created
    if (!cloud->empty()){
      // allocate if neccessary
      if (d_depth.size()!=cv::Size(pc_rows, pc_cols) || d_depth.depth()!= CV_64F)
	d_depth = cv::Mat::zeros(pc_rows, pc_cols, CV_64F);
      else
	d_depth = cv::Scalar(10.0);

      if (d_dmask.size()!=cv::Size(pc_rows, pc_cols) || d_dmask.depth()!= CV_8U)
	d_dmask = cv::Mat::zeros(pc_rows, pc_cols, CV_8U);
      else
	d_dmask = cv::Scalar(0);

      for (int r=0; r<pc_rows; r++){
	double* depth_r = d_depth.ptr<double> (r);
	unsigned char* dmask_r = d_dmask.ptr<unsigned char> (r);

	for (int c=0; c<pc_cols; c++){
	  PointX point = cloud->at(c,r);
	  //set depth and depth-mask if not NaN
	  // if (!isnan(point.z) && point.z<4.0 && point.z>0.6){
	  if (!isnan(point.z)){
	    depth_r[c] = (double)point.z;
	    dmask_r[c] = (unsigned char)255;
	  }
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

void shr_cv_utils::toTransMat(const Eigen::Quaterniond quat,
		const Eigen::Vector3d trans,
		Eigen::Matrix4f &trans_mat)
{
  trans_mat.setZero();
  trans_mat.topLeftCorner(3,3) = quat.toRotationMatrix().cast<float>();
  trans_mat.topRightCorner(3,1) = trans.cast<float>();
  trans_mat(3,3) = 1.0;
}

void shr_cv_utils::transPoints(const PointCloudT::Ptr pc_in, 
			       const Eigen::Matrix4f &trans, 
			       PointCloudT::Ptr &pc_out)
{
  Eigen::MatrixXf m(4,(*pc_in).size());
    for(size_t i=0;i<(*pc_in).size();i++) {
      m(0,i) = (*pc_in)[i].x; m(1,i) = (*pc_in)[i].y; m(2,i) = (*pc_in)[i].z; m(3,i) = 1;
    }
    m = trans * m;
    for(size_t i=0;i<(*pc_in).size();i++) {
        PointT pt;
        pt.x = m(0,i); pt.y = m(1,i); pt.z = m(2,i); pt.rgb = (*pc_in)[i].rgb;
        (*pc_out).push_back(pt);
    }
}
