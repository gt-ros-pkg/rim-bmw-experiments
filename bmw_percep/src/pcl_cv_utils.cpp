#include<bmw_percep/pcl_cv_utils.hpp>

/**
   Utility functions for OpenCV and PCL
**/

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PointCloudX;

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
	  if (!isnan(point.z) && point.z<4.0 && point.z>0.6){
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

bool cv_utils::pc_to_depth(const PointCloudX::Ptr& cloud, 
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
	  if (!isnan(point.z) && point.z<4.0 && point.z>0.6){
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


// Find blobs in a binary image as a vector of a vector of 2D points
// returns the area of largest blob
int cv_utils::find_blobs(const cv::Mat &binary, vector < vector<cv::Point2i> > &blobs)
{
  blobs.clear();
  int max_area =0;
  int max_label =-1;
  vector<int> label_areas;
  label_areas.clear();
  
  cv::Mat label_img;
  binary.convertTo(label_img, CV_32FC1);
    
  float label_count = 2; // starts at 2 because 0,1 are used already
  
  for(int y=0; y < label_img.rows; y++) {
    const float* label_row = label_img.ptr<float>(y);
    for(int x=0; x < label_img.cols; x++) {
      if(int(label_row[x]) !=1) {continue;}
      cv::Rect rect;
      cv::floodFill(label_img, cv::Point(x,y), cv::Scalar(label_count), &rect);
	
      vector <cv::Point2i> blob;
      blob.clear();

      int label_area=0;
      
      for(int i=rect.y; i < (rect.y+rect.height); i++) {
	for(int j=rect.x; j < (rect.x+rect.width); j++) {
	  if(int(label_img.at<float>(i,j)) != label_count) {continue;}
	  blob.push_back(cv::Point2i(j,i));
	  label_area++;
	}
      }

      label_areas.push_back(label_area);
      blobs.push_back(blob);
	
      //check for max
      if (label_area>max_area)
	{
	  max_area=label_area; 
	  max_label=label_count-2;
	}
	
      label_count++;
    }
  }

  //return the maximum area found under a blob
  return max_area;
}

// Find blobs in a depth image as a vector of a vector of 2D points
// returns the area of largest blob
int cv_utils::find_blobs_depth(const cv::Mat &binary, const cv::Mat &depth,
		     vector < vector<cv::Point2i> > &blobs, float depth_delta)
{
  blobs.clear();
  int max_area =0;
  int max_label =-1;
  vector<int> label_areas;
  label_areas.clear();
  
  cv::Mat label_img; label_img = cv::Mat::zeros(binary.size(), depth.depth());
  depth.copyTo(label_img, binary);
    
  float label_count = 8.0; // starts at 8 because 4m max-range
  
  for(int y=0; y < label_img.rows; y++) {
    const float* label_row = label_img.ptr<float>(y);
    for(int x=0; x < label_img.cols; x++) {
      if(label_row[x]==0.0 || label_row[x]>7.0) {continue;}
      cv::Rect rect;
      cv::floodFill(label_img, cv::Point(x,y), cv::Scalar(label_count), &rect, 
		    depth_delta, depth_delta);
      
      vector <cv::Point2i> blob;
      blob.clear();

      int label_area=0;
      
      for(int i=rect.y; i < (rect.y+rect.height); i++) {
	for(int j=rect.x; j < (rect.x+rect.width); j++) {
	  if(int(label_img.at<float>(i,j)) != label_count) {continue;}
	  blob.push_back(cv::Point2i(j,i));
	  label_area++;
	}
      }

      label_areas.push_back(label_area);
      blobs.push_back(blob);
	
      //check for max
      if (label_area>max_area)
	{
	  max_area=label_area; 
	  max_label=label_count-2;
	}
	
      label_count++;
    }
  }

  //return the maximum area found under a blob
  return max_area;
}
