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

//Extract RGB, Depth, and Valid-depth maps from point cloud
bool cv_utils::pc_to_img( PointCloudT::ConstPtr& cloud, cv::Mat& d_rgb, 
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

//Extract RGB, Depth, and Valid-depth maps from point cloud
bool cv_utils::pc_to_img_no_filter( const PointCloudT::Ptr& cloud, cv::Mat& d_rgb, 
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
	    depth_r[c] = static_cast<double>(point.z);
	    dmask_r[c] = static_cast<unsigned char>(255);
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

bool cv_utils::pc_to_img_no_filter( PointCloudT::ConstPtr& cloud, cv::Mat& d_rgb, 
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
	    depth_r[c] = static_cast<double>(point.z);
	    dmask_r[c] = static_cast<unsigned char>(255);
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


// Find blobs in a binary image as a vector of a vector of 2D points
// returns the area of largest blob
int cv_utils::find_blobs(const cv::Mat &binary, vector < vector<cv::Point2i> > &blobs,
			 int& max_blob_id)
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
  max_blob_id = max_label;
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
      // TODO: Fix the 7 problem, if more than 7 blobs then this obviously wrong
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
	  //TODO: Check if correct
	  max_label=label_count-8.0f;
	}
	
      label_count++;
    }
  }

  //return the maximum area found under a blob
  return max_area;
}

void cv_utils::find_euclid_blobs(PointCloudX::ConstPtr cloud, 
				 pcl::PointCloud<pcl::PointXYZRGB>::Ptr viz_cloud, 
				 const cv::Mat &mask, 
				 vector<cv::Point3f> clusters, int& max_blob_id,
				 float leaf_size/*=0.01*/)
{
  //Remove masked points
  PointCloudX::Ptr cloud_unmasked(new PointCloudX);
  //copy over unmasked points
  int pc_rows=cloud->height;
  int pc_cols=cloud->width;
  
  cloud_unmasked->height = cloud->height;
  cloud_unmasked->width = cloud->width;
  cloud_unmasked->is_dense = cloud->is_dense;
  //cloud_unmasked->sensor_origin = cloud->sensor_origin;
  //cloud_unmasked->sensor_orientation = cloud->sensor_orientation;

  //start adding them points
  for (int r=0; r<pc_rows; r++){
    const double* mask_r = mask.ptr<double> (r);
    for (int c=0; c<pc_cols; c++){
      if (mask_r[c]>0){
 	PointX point = cloud->at(c,r);
 	cloud_unmasked->points.push_back(point);
      }
    }
  }

  

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointX> vg;
  PointCloudX::Ptr cloud_filtered(new PointCloudX);
  vg.setInputCloud (cloud_unmasked);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 

 // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;
  // visualize by painting each PC another color
  viz_cloud->points.clear();
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
       it != cluster_indices.end (); ++it){
    for (std::vector<int>::const_iterator pit = it->indices.begin (); 
 	 pit != it->indices.end (); pit++){
      // create RGB point to push in
      pcl::PointXYZRGB new_pt;
      new_pt.x = cloud->points[*pit].x;
      new_pt.y = cloud->points[*pit].y;
      new_pt.z = cloud->points[*pit].z;
      uint8_t r(255), g(255), b(255);
      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
      new_pt.rgb = *reinterpret_cast<float*>(&rgb);
      viz_cloud->points.push_back (new_pt); //*
    }
    viz_cloud->width = viz_cloud->points.size ();
    viz_cloud->height = 1;
    viz_cloud->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << viz_cloud->points.size () << " data points." << std::endl;
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    j++;

  }

}

void cv_utils::find_euclid_blobs(PointCloudX::ConstPtr cloud, 
				 pcl::PointCloud<pcl::PointXYZ>::Ptr viz_cloud, 
				 const cv::Mat &mask, 
				 vector<cv::Point3f> clusters, int& max_blob_id,
				 float leaf_size/*=0.01*/)
{
  //Remove masked points
  PointCloudX::Ptr cloud_unmasked(new PointCloudX);
  //copy over unmasked points
  int pc_rows=cloud->height;
  int pc_cols=cloud->width;
  
  //cloud_unmasked->sensor_origin = cloud->sensor_origin;
  //cloud_unmasked->sensor_orientation = cloud->sensor_orientation;
  cloud_unmasked->points.clear();
  //start adding them points
  for (int r=0; r<pc_rows; r++){
    const double* mask_r = mask.ptr<double> (r);
    for (int c=0; c<pc_cols; c++){
      if (mask_r[c]>0){
 	PointX point = cloud->at(c,r);
 	cloud_unmasked->points.push_back(point);
      }
    }
  }

  cloud_unmasked->height = 1;
  cloud_unmasked->width = cloud_unmasked->points.size();
  cloud_unmasked->is_dense = true;
  
  //copyPointCloud(*cloud_unmasked, *viz_cloud);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointX> vg;
  PointCloudX::Ptr cloud_filtered(new PointCloudX);
  vg.setInputCloud (cloud_unmasked);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 

  copyPointCloud(*cloud_filtered, *viz_cloud);
  return;
  

 // // Creating the KdTree object for the search method of the extraction
 //  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
 //  tree->setInputCloud (cloud_filtered);

 //  std::vector<pcl::PointIndices> cluster_indices;
 //  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
 //  ec.setClusterTolerance (0.02); // 2cm
 //  ec.setMinClusterSize (100);
 //  ec.setMaxClusterSize (25000);
 //  ec.setSearchMethod (tree);
 //  ec.setInputCloud (cloud_filtered);
 //  ec.extract (cluster_indices);

 //  int j = 0;
 //  // visualize by painting each PC another color
 //  viz_cloud->points.clear();
 //  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
 //       it != cluster_indices.end (); ++it){
 //    for (std::vector<int>::const_iterator pit = it->indices.begin (); 
 // 	 pit != it->indices.end (); pit++){
 //      // create RGB point to push in
 //      pcl::PointXYZRGB new_pt;
 //      new_pt.x = cloud->points[*pit].x;
 //      new_pt.y = cloud->points[*pit].y;
 //      new_pt.z = cloud->points[*pit].z;
 //      uint8_t r(255), g(255), b(255);
 //      uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
 //              static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
 //      new_pt.rgb = *reinterpret_cast<float*>(&rgb);
 //      viz_cloud->points.push_back (new_pt); //*
 //    }
 //    viz_cloud->width = viz_cloud->points.size ();
 //    viz_cloud->height = 1;
 //    viz_cloud->is_dense = true;

 //    std::cout << "PointCloud representing the Cluster: " << viz_cloud->points.size () << " data points." << std::endl;
 //    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
 //    j++;

 //  }

 }

void cv_utils::find_euclid_blobs(PointCloudT::ConstPtr cloud, 
				 PointCloudT::Ptr viz_cloud, 
				 vector<cv::Point3f> clusters, int& max_blob_id,
				 const Eigen::VectorXf ground_coeffs,
				 float leaf_size/*=0.01*/)
{

  float voxel_size=0.06;

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointT> vg;
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.06f, 0.06f, 0.06f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 

  // Ground removal and update:
  pcl::IndicesPtr inliers(new std::vector<int>);
  boost::shared_ptr<pcl::SampleConsensusModelPlane<PointT> > ground_model(new pcl::SampleConsensusModelPlane<PointT>(cloud_filtered));
  ground_model->selectWithinDistance(ground_coeffs, 3*voxel_size, *inliers);
  PointCloudT::Ptr no_ground_cloud(new PointCloudT);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*no_ground_cloud);
  
  // pcl::copyPointCloud(*no_ground_cloud, *viz_cloud);
  // return;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (no_ground_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (2* 0.06); // 2cm
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (5000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (no_ground_cloud);
  ec.extract (cluster_indices);

  std::vector<pcl::people::PersonCluster<PointT> > ppl_clusters; 

  find_ppl_clusters(no_ground_cloud,
  		    cluster_indices,
  		    ppl_clusters,
  		    ground_coeffs);

  //debug
  cout << "No. of clusters: " << cluster_indices.size() << endl;

  int j = 0;
  
  //replace cluster indices with people clusters
  int n_ppl=0;
  cluster_indices.clear();
  for(std::vector<pcl::people::PersonCluster<PointT> >::iterator 
  	it = ppl_clusters.begin(); it != ppl_clusters.end(); ++it)
      {
  	cluster_indices.push_back(it->getIndices());
  	n_ppl++;
      }
  
  cout << "No. of people: " << n_ppl << endl; 
  // string whatupp;
  // if (n_ppl > 100)
  //   cin >> whatupp;

  // visualize by painting each PC another color
  pcl::copyPointCloud(*no_ground_cloud, *cloud_filtered);
  
  viz_cloud->points.clear();
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
       it != cluster_indices.end (); ++it){
    uint8_t r(rand()%255), g(rand()%255), b(rand()%255);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); 
 	 pit != it->indices.end (); pit++){
      // create RGB point to push in
      PointT new_pt;
      new_pt.x = cloud_filtered->points[*pit].x;
      new_pt.y = cloud_filtered->points[*pit].y;
      new_pt.z = cloud_filtered->points[*pit].z;

      // new_pt.r = cloud_filtered->points[*pit].r;
      // new_pt.g = cloud_filtered->points[*pit].g;
      // new_pt.b = cloud_filtered->points[*pit].b;

      new_pt.r = r; new_pt.g = g; new_pt.b = b;
      new_pt.a = 1.0;
      viz_cloud->points.push_back (new_pt); //*
    }
    viz_cloud->width = viz_cloud->points.size ();
    viz_cloud->height = 1;
    viz_cloud->is_dense = true;

    //std::cout << "PointCloud representing the Cluster: " << viz_cloud->points.size () << " data points." << std::endl;
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    //j++;

  }
}

void cv_utils::find_ppl_clusters(const PointCloudT::Ptr cloud, 
		  vector<pcl::PointIndices>& init_indices, 
		  std::vector<pcl::people::PersonCluster<PointT> >& clusters,
		  const Eigen::VectorXf ground_coeffs)
{
  
  //debug
  // cout << "Started this finding.." << endl;

  float max_height_= 2.3; float min_height_= 1.3;
  int min_a_merge = 200; float max_dist_gr=0.4;
  bool camera_vertical=false, compute_head=false;

  float sqrt_ground_coeffs_ = (ground_coeffs - Eigen::Vector4f
				(0.0f, 0.0f, 0.0f, ground_coeffs(3))).norm();

  // Person clusters creation from clusters indices:
  for(std::vector<pcl::PointIndices>::const_iterator it = 
	init_indices.begin(); it != init_indices.end(); ++it)
    {
      pcl::people::PersonCluster<PointT> cluster(cloud, *it, ground_coeffs, sqrt_ground_coeffs_, compute_head, camera_vertical); // PersonCluster creation
      clusters.push_back(cluster);
    }

  //debug
  // cout << "Created ppl clusters.." << endl;

  // Remove clusters with too high height from the ground plane:
  std::vector<pcl::people::PersonCluster<PointT> > new_clusters;
  for(unsigned int i = 0; i < clusters.size(); i++) // for every cluster
    {
      if (clusters[i].getHeight() <= max_height_)
	new_clusters.push_back(clusters[i]);
    }

  //Merge clusters_close in floor coordinates:
  clusters.clear();
  mergeClustersCloseInFloorCoordinates(cloud, new_clusters, clusters,
				       ground_coeffs, sqrt_ground_coeffs_);

  // Remove clusters far away from ground or too short
  new_clusters.clear();
  for(unsigned int i = 0; i < clusters.size(); i++) // for every cluster
    {
      if (clusters[i].getNumberPoints() > min_a_merge){
	if (clusters[i].getHeight() >= min_height_){
	  if (get_min_ground_dist(cloud, clusters[i], ground_coeffs, 
				  sqrt_ground_coeffs_, max_dist_gr))
  	    new_clusters.push_back(clusters[i]);
	}
      }
    }

  // //debug
  // cout << "Donadone.." << endl;
  clusters.clear();
  clusters = new_clusters;
  return;
  
  new_clusters.clear();

  // std::vector<pcl::people::PersonCluster<PointT> > subclusters;
  // int cluster_min_points_sub = int(float(min_points_) * 1.5);
}

void cv_utils::mergeClustersCloseInFloorCoordinates 
( const PointCloudT::Ptr cloud, 
std::vector<pcl::people::PersonCluster<PointT> >& input_clusters,
 std::vector<pcl::people::PersonCluster<PointT> >& output_clusters, 
 const Eigen::VectorXf ground_coeffs_, double sqrt_ground_coeffs_)
{
  float min_distance_between_cluster_centers = 0.4; // meters
  float normalize_factor = std::pow(sqrt_ground_coeffs_, 2); // sqrt_ground_coeffs
							     // ^ 2
							     // (precomputed
							     // for
							     // speed)

  Eigen::Vector3f head_ground_coeffs = ground_coeffs_.head(3); // ground
							      // plane
							      // normal
							      // (precomputed
							      // for
							      // speed)
  std::vector <std::vector<int> > connected_clusters;
  connected_clusters.resize(input_clusters.size());
  std::vector<bool> used_clusters; // 0 in correspondence of clusters
				   // remained to process, 1 for
				   // already used clusters
  used_clusters.resize(input_clusters.size());
  // initialize clusters unused
  for(vector<bool>::iterator usit=used_clusters.begin(); usit!=used_clusters.end();
      usit++)
    *usit = false;
    
  for(unsigned int i = 0; i < input_clusters.size(); i++) // for every cluster
    {
      Eigen::Vector3f theoretical_center = input_clusters[i].getTCenter();
      float t = theoretical_center.dot(head_ground_coeffs) /
	normalize_factor; // height from the ground
      Eigen::Vector3f current_cluster_center_projection = 
	theoretical_center - head_ground_coeffs * t; // projection of
						     // the point on
						     // the
						     // groundplane
      for(unsigned int j = i+1; j < input_clusters.size(); j++) // for
								// every
								// remaining
								// cluster
	{
	  theoretical_center = input_clusters[j].getTCenter();
	  float t = theoretical_center.dot(head_ground_coeffs) 
	    / normalize_factor; // height from the ground
	  Eigen::Vector3f new_cluster_center_projection =
	    theoretical_center - head_ground_coeffs * t; // projection
							 // of the
							 // point on
							 // the
							 // groundplane
	  if (((new_cluster_center_projection - 
		current_cluster_center_projection).norm()) < 
	      min_distance_between_cluster_centers)
	    {
	      connected_clusters[i].push_back(j);
	    }
	}
    }

 for(unsigned int i = 0; i < connected_clusters.size(); i++) // for every cluster
  {
    if (!used_clusters[i]) // if this cluster has not been used yet
    {
      used_clusters[i] = true;
      if (connected_clusters[i].empty()) // no other clusters to merge
      {
        output_clusters.push_back(input_clusters[i]);
      }
      else
      {
        // Copy cluster points into new cluster:
        pcl::PointIndices point_indices;
        point_indices = input_clusters[i].getIndices();
        for(unsigned int j = 0; j < connected_clusters[i].size(); j++)
        {
          if (!used_clusters[connected_clusters[i][j]]) // if this
							// cluster has
							// not been
							// used yet
          {
            used_clusters[connected_clusters[i][j]] = true;
            for(std::vector<int>::const_iterator points_iterator = 
		  input_clusters[connected_clusters[i][j]].getIndices().
		  indices.begin();
                points_iterator != 
		  input_clusters[connected_clusters[i][j]].getIndices().
		  indices.end(); points_iterator++)
            {
              point_indices.indices.push_back(*points_iterator);
            }
          }
        }
        pcl::people::PersonCluster<PointT> cluster(cloud, point_indices, ground_coeffs_, sqrt_ground_coeffs_, false, false);
        output_clusters.push_back(cluster);
      }
    }
  }
}

bool cv_utils::get_min_ground_dist (const PointCloudT::Ptr cloud, 
				    pcl::people::PersonCluster<PointT> person_c, 
				    const Eigen::VectorXf ground_coeffs, 
				    double sqrt_ground_coeffs, double min_dist)
{
  pcl::PointIndices p_ind = person_c.getIndices();
  double min_disto=1000.0;

  for (std::vector<int>::const_iterator it = p_ind.indices.begin (); 
       it != p_ind.indices.end(); ++it){
    Eigen::Vector4f bott_pt;
    bott_pt << cloud->points[*it].x, cloud->points[*it].y, cloud->points[*it].z, 1.0f;
    double dist = fabs(bott_pt.dot(ground_coeffs)/sqrt_ground_coeffs);
    if (min_disto > dist)
      min_disto = dist;
    if (min_dist >= dist)
      return true;
  }
  
  cout << "Minimum cluster distance is : " << min_disto << endl;
  return false;
  
}


void cv_utils::find_euclid_blobs(PointCloudT::ConstPtr cloud, 
				 PointCloudT::Ptr viz_cloud, 
				 vector<cv::Point3f> clusters, int& max_blob_id,
				 const Eigen::VectorXf ground_coeffs, cv::Mat bg,
				 float leaf_size/*=0.01*/)
{

  // background subtract the point cloud
  PointCloudT::Ptr bgCloud(new PointCloudT);
  depth_bgSub(cloud, bgCloud, bg);

  //debug
  cout << " Get here" << endl;
  
  // pcl::copyPointCloud(*bgCloud, *viz_cloud);
  // return;
  
  float voxel_size=0.06;
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointT> vg;
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  vg.setInputCloud (bgCloud);
  vg.setLeafSize (0.06f, 0.06f, 0.06f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 

  // TODO: Check background subtraction
  // pcl::copyPointCloud(*bgCloud, *viz_cloud);
  // return;

  // Ground removal and update:
  pcl::IndicesPtr inliers(new std::vector<int>);
  boost::shared_ptr<pcl::SampleConsensusModelPlane<PointT> > ground_model(new pcl::SampleConsensusModelPlane<PointT>(cloud_filtered));
  ground_model->selectWithinDistance(ground_coeffs, voxel_size, *inliers);
  PointCloudT::Ptr no_ground_cloud(new PointCloudT);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*no_ground_cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (no_ground_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (2* 0.06); // 2cm
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (5000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (no_ground_cloud);
  ec.extract (cluster_indices);

  std::vector<pcl::people::PersonCluster<PointT> > ppl_clusters; 

  find_ppl_clusters(no_ground_cloud,
  		    cluster_indices,
  		    ppl_clusters,
  		    ground_coeffs);

  //debug
  cout << "No. of clusters: " << cluster_indices.size() << endl;

  int j = 0;
  
  //replace cluster indices with people clusters
  int n_ppl=0;
  cluster_indices.clear();
  for(std::vector<pcl::people::PersonCluster<PointT> >::iterator 
  	it = ppl_clusters.begin(); it != ppl_clusters.end(); ++it)
      {
  	cluster_indices.push_back(it->getIndices());
  	n_ppl++;
      }
  
  cout << "No. of people: " << n_ppl << endl; 
  // string whatupp;
  // if (n_ppl > 100)
  //   cin >> whatupp;

  // visualize by painting each PC another color
  pcl::copyPointCloud(*no_ground_cloud, *cloud_filtered);
  
  viz_cloud->points.clear();
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
       it != cluster_indices.end (); ++it){
    uint8_t r(rand()%255), g(rand()%255), b(rand()%255);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); 
 	 pit != it->indices.end (); pit++){
      // create RGB point to push in
      PointT new_pt;
      new_pt.x = cloud_filtered->points[*pit].x;
      new_pt.y = cloud_filtered->points[*pit].y;
      new_pt.z = cloud_filtered->points[*pit].z;

      new_pt.r = cloud_filtered->points[*pit].r;
      new_pt.g = cloud_filtered->points[*pit].g;
      new_pt.b = cloud_filtered->points[*pit].b;

      // new_pt.r = r; new_pt.g = g; new_pt.b = b;
      new_pt.a = 1.0;
      viz_cloud->points.push_back (new_pt); //*
    }
    viz_cloud->width = viz_cloud->points.size ();
    viz_cloud->height = 1;
    viz_cloud->is_dense = true;

    //std::cout << "PointCloud representing the Cluster: " << viz_cloud->points.size () << " data points." << std::endl;
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    //j++;

  }
  
}

void cv_utils::find_euclid_blobs(PointCloudT::ConstPtr cloud, 
				 PointCloudT::Ptr viz_cloud, 
				 vector<cv::Point3f> clusters, int& max_blob_id,
				 const Eigen::VectorXf ground_coeffs, 
				 cv::BackgroundSubtractorMOG2 cvBg,
				 float leaf_size/*=0.01*/)
{

  // background subtract the point cloud
  PointCloudT::Ptr bgCloud(new PointCloudT);
  depth_bgSub(cloud, bgCloud, cvBg);
  
  // pcl::copyPointCloud(*bgCloud, *viz_cloud);
  // return;
  
  float voxel_size=0.06;
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<PointT> vg;
  PointCloudT::Ptr cloud_filtered(new PointCloudT);
  vg.setInputCloud (bgCloud);
  vg.setLeafSize (0.06f, 0.06f, 0.06f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; 

  // TODO: Check background subtraction
  // pcl::copyPointCloud(*bgCloud, *viz_cloud);
  // return;

  // Ground removal and update:
  pcl::IndicesPtr inliers(new std::vector<int>);
  boost::shared_ptr<pcl::SampleConsensusModelPlane<PointT> > ground_model(new pcl::SampleConsensusModelPlane<PointT>(cloud_filtered));
  ground_model->selectWithinDistance(ground_coeffs, voxel_size, *inliers);
  PointCloudT::Ptr no_ground_cloud(new PointCloudT);
  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*no_ground_cloud);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (no_ground_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (2* 0.06); // 2cm
  ec.setMinClusterSize (30);
  ec.setMaxClusterSize (5000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (no_ground_cloud);
  ec.extract (cluster_indices);

  std::vector<pcl::people::PersonCluster<PointT> > ppl_clusters; 

  find_ppl_clusters(no_ground_cloud,
  		    cluster_indices,
  		    ppl_clusters,
  		    ground_coeffs);

  //debug
  cout << "No. of clusters: " << cluster_indices.size() << endl;

  int j = 0;
  
  //replace cluster indices with people clusters
  int n_ppl=0;
  cluster_indices.clear();
  for(std::vector<pcl::people::PersonCluster<PointT> >::iterator 
  	it = ppl_clusters.begin(); it != ppl_clusters.end(); ++it)
      {
  	cluster_indices.push_back(it->getIndices());
  	n_ppl++;
      }
  
  cout << "No. of people: " << n_ppl << endl; 
  // string whatupp;
  // if (n_ppl > 100)
  //   cin >> whatupp;

  // visualize by painting each PC another color
  pcl::copyPointCloud(*no_ground_cloud, *cloud_filtered);
  
  viz_cloud->points.clear();
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); 
       it != cluster_indices.end (); ++it){
    uint8_t r(rand()%255), g(rand()%255), b(rand()%255);

    for (std::vector<int>::const_iterator pit = it->indices.begin (); 
 	 pit != it->indices.end (); pit++){
      // create RGB point to push in
      PointT new_pt;
      new_pt.x = cloud_filtered->points[*pit].x;
      new_pt.y = cloud_filtered->points[*pit].y;
      new_pt.z = cloud_filtered->points[*pit].z;

      // new_pt.r = cloud_filtered->points[*pit].r;
      // new_pt.g = cloud_filtered->points[*pit].g;
      // new_pt.b = cloud_filtered->points[*pit].b;

      new_pt.r = r; new_pt.g = g; new_pt.b = b;
      new_pt.a = 1.0;
      viz_cloud->points.push_back (new_pt); //*
    }
    viz_cloud->width = viz_cloud->points.size ();
    viz_cloud->height = 1;
    viz_cloud->is_dense = true;

    //std::cout << "PointCloud representing the Cluster: " << viz_cloud->points.size () << " data points." << std::endl;
    //writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
    //j++;

  }
  
}

void cv_utils::depth_bgSub( PointCloudT::ConstPtr cloud, PointCloudT::Ptr bgCloud, 
		 const cv::Mat& bg)
{
  double max_diff = 0.1;
  int pc_rows=cloud->height;
  int pc_cols=cloud->width;
  cv::Mat rgb_im, depth_im, depth_mask, depth_show;

  // //debug
  // cv::Mat bg_;
  // bg.copyTo(bg_);
  // //generate same depth image
  // for (int r=0; r<pc_rows; r++){
  //   double* bg_r_ = bg_.ptr<double> (r);
  //   for (int c=0; c<pc_cols; c++){
  //     PointT point = cloud->at(c,r);
  //     bg_r_[c] = point.z;
  //   }
  // }
  

  bgCloud->points.clear();

  pc_to_img_no_filter(cloud, rgb_im, depth_im, depth_mask);

  // //check if conversion possible
  // if (cloud->isOrganized()){
  //   //TODO: Check if allocated Mat can be created
  //   if (!cloud->empty()){
  //     for (int r=0; r<pc_rows; r++){
  // 	const double* bg_r = bg.ptr<double> (r);
  // 	for (int c=0; c<pc_cols; c++){
  // 	  PointT point = cloud->at(c,r);
  // 	  if (!isnan(point.z) && point.z<6.0 && point.z>0.5){
  // 	    //set depth and depth-mask if not NaN
  // 	    double bg_diff = fabs(point.z-bg_r[c]);
  // 	    if (bg_diff > max_diff)
  // 	      bgCloud->points.push_back(point);
  // 	  }
  // 	}
  //     }
  //   }
  //   else{
  //     cout << "\n Cloud Empty!" << endl;
  //     return ;
  //   }
  // }
  // else{
  //   cout << endl << "Cloud Unorganized.." << endl;
  //   return;
  // }

  for(int r=0; r<depth_im.rows; r++){
      const double* depth_r = depth_im.ptr<double> (r);
      const double* bg_r = bg.ptr<double> (r);
      const uchar* dmask_r = depth_mask.ptr<uchar> (r);

      for(int c=0; c<depth_im.cols; c++){
	if (dmask_r[c]>0){
	  if (!isnan(depth_r[c]) && depth_r[c]>0.5 && depth_r[c]<5.0){
	    if (fabs(depth_r[c]-bg_r[c]) > 0.1){
	      PointT pointy = cloud->at(c,r);
	      bgCloud->points.push_back(pointy);
	    }
	  }
	}
      }
    }  

  bgCloud->width = bgCloud->points.size();
  bgCloud->height = 1;
  bgCloud->is_dense = true;
  
  return;
}


void cv_utils::depth_bgSub( PointCloudT::ConstPtr cloud, PointCloudT::Ptr bgCloud, 
			    cv::BackgroundSubtractorMOG2 cvBg)
{
  double general_learn=0.0;
  int pc_rows=cloud->height;
  int pc_cols=cloud->width;

  cv::Mat rgb_im, depth_im, depth_mask, foreMask, depth_filt;

  cv_utils::pc_to_img(cloud, rgb_im, depth_im, depth_mask);
  
  //subtract-bg
  cvBg.operator()(depth_im, foreMask, general_learn);

  bgCloud->points.clear();
  
  //check if conversion possible
  if (cloud->isOrganized()){

    if (!cloud->empty()){
      for (int r=0; r<pc_rows; r++){
	const double* depth_r = depth_im.ptr<double> (r);
	const double* fore_r = foreMask.ptr<double> (r);
	for (int c=0; c<pc_cols; c++){
	  if (fore_r > 0){
	    if (!isnan(depth_r[c]) && depth_r[c]<6.0 && depth_r[c]>0.5){
	      PointT point = cloud->at(c,r);
	      bgCloud->points.push_back(point);
	    }
	  }
	}
      }
    }
    else{
      cout << "\n Cloud Empty!" << endl;
      return ;
    }
  }
  else{
    cout << endl << "Cloud Unorganized.." << endl;
    return;
  }
  

  bgCloud->width = bgCloud->points.size();
  bgCloud->height = 1;
  bgCloud->is_dense = true;
  
  return;
}
