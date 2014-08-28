#include <bmw_percep/groundPlane.hpp>

/**
   
   Class *implementation* for estimating the ground/floor plane from RGBD imagery
   Uses PCL.
   
**/

using namespace std;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// struct for passing arguments to mouse-callback
typedef struct callback_args_{
  vector<cv::Point>* plane_im_pts;
  cv::Mat img;
} callback_args;

void paint_at_point(cv::Mat img, cv::Point loc, cv::Scalar col, int diam=1)
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

void paint_at_point(cv::Mat& img, cv::Point loc, const cv::Mat& orig, int diam=1)
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

static void onClick(int event, int x, int y, int,  void* param)
{
  //convert param
  callback_args* c_arg = (callback_args*) param;

  if (event != cv::EVENT_LBUTTONDOWN)
    return;

  cv::Point push_pt = cv::Point(x,y);
  
  c_arg->plane_im_pts->push_back(push_pt);
  
  //paint point and show
  cv::Scalar pt_recolor = cv::Scalar(0, 255, 0);
  c_arg->img.at<cv::Scalar>(y,x) = pt_recolor;
  paint_at_point(c_arg->img, push_pt, pt_recolor);
  
  //debug
  cout << endl << "Click" << endl;
  cout << "Point: " << push_pt<< endl;
  cout << "This was pushed:" << c_arg->plane_im_pts->back() << endl;
  
  imshow("Click Points", c_arg->img);
  return;
}

//convert a point cloud to Mat
cv::Mat pc_to_img(const PointCloudT::Ptr& cloud, bool mask_nans=true)
{
  cv::Mat result;
  int pc_rows=cloud->height;
  int pc_cols=cloud->width;

  //check if conversion possible
  if (cloud->isOrganized())
    {
      result = cv::Mat::zeros(pc_rows, pc_cols, CV_8UC3);
      
      if (!cloud->empty())
	{
	  for (int r=0; r<pc_rows; r++){
	    cv::Vec3b* res_i = result.ptr<cv::Vec3b>(r);
	    for (int c=0; c<pc_cols; c++){
	      PointT point = cloud->at(c,r);
	      //check if NaNs - i.e. invalid depth
	      if (mask_nans){
		if (isnan(point.z))
		  continue; // set zero instead of color
	      }
	      Eigen::Vector3i rgb = point.getRGBVector3i();
	      res_i[c][0] = rgb[2];
	      res_i[c][1] = rgb[1];
	      res_i[c][2] = rgb[0];
	    }
	  }
	}
    }
  else{
    cout << endl << "Cloud Unorganized.." << endl;
  }

  return result;
}

GroundPlane::GroundPlane(const PointCloudT::Ptr& cloud)
{
  //TODO: How stupid is not mentioning what different buttons do.
  //Print the message

  //Instructions
  cout << "\n**********Interaction Instructions**********\n";
  cout << "1. Click on the image to select points. \n";
  cout << "2. Press 'd' to delete last point.\n";
  cout << "3. Press 'Esc' to compute plane." << endl;
    

  //resize plane parameters
  ground_coeffs.resize(4);

  // convert pointcloud to rgb image
  const cv::Mat pc_im = pc_to_img(cloud);
  cv::Mat disp_im;
  pc_im.copyTo(disp_im);

  //window and callbacks
  cv::namedWindow("Click Points", cv::WINDOW_AUTOSIZE);
  vector<cv::Point> ground_pts_2d;
  ground_pts_2d.clear();
  
  // user callback arguments
  callback_args c_arg;
  c_arg.plane_im_pts = &ground_pts_2d;
  c_arg.img = disp_im;

  cv::setMouseCallback("Click Points", onClick, &c_arg);
  char c=0; // key-press

  for(;;) { // Quit when press Esc key

    imshow("Click Points", disp_im);
    c = cv::waitKey(0);
    if (c=='d')  {// delete last point
      if (ground_pts_2d.size() > 0){
	cv::Point del_pt = ground_pts_2d.back();
	ground_pts_2d.pop_back();
	cout << "\nDeleted Last Point: " << del_pt << endl;
	paint_at_point(disp_im, del_pt, pc_im);
      }
      else
	cout << "\nNo point to delete\n";
    }

    // can't quit with less than 3 points
    if (c==27){
      if(ground_pts_2d.size()<3){
	cout << "\nCan not quit before choosing 3 points..." <<endl;
	c = 0;
      }
      else{

	if (!compute_plane_dlt(cloud, ground_pts_2d)){
	  //if plane can't be computed correctly
	  cout << "\nThe plane wasn't estimated correctly. Label points again.."
	       << endl ;
	  //Refresh
	  ground_pts_2d.clear();
	  pc_im.copyTo(disp_im);
	}
	else{
	  //break out if Esc pressed and plane correctly computed
	  break;
	}
      }
    }
  }
  cv::destroyWindow("Click Points");

  //debug
  cout << "\nPlane Parameters:-\n" << "a: " << ground_coeffs.at(0) 
       << "\t b: " << ground_coeffs.at(1) 
       << "\t c: " << ground_coeffs.at(2) 
       << "\t d: " << ground_coeffs.at(3) << endl;

  //debug
  visualizePlane(cloud);
  

  return;
}

bool GroundPlane::compute_plane_dlt(const PointCloudT::Ptr& cloud, 
				    vector<cv::Point> pts_2d)
{
  const int mat_cols=4;
  Eigen::MatrixXf pt_mat(pts_2d.size(), mat_cols);

  // convert 2D points to 3D by accessing point-cloud
  if (!cloud->empty()){
    for (int i=0; i<pts_2d.size(); i++){
      const cv::Point pt_2d = pts_2d.at(i);
      const PointT pt_3d = cloud->at(pt_2d.x, pt_2d.y);
      
      //set mat-row
      pt_mat(i,0) = pt_3d.x;
      pt_mat(i,1) = pt_3d.y;
      pt_mat(i,2) = pt_3d.z;
      pt_mat(i,3) = 1.0;
    }
    
    //debug
    cout << endl << "---------Point Matrix----------\n" << pt_mat << endl; 
    
    //SVD 'Trick'
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(pt_mat, Eigen::ComputeFullV);
    //Get rightmost column from the V matrix
    Eigen::Vector4f v_l_col = svd.matrixV().rightCols(1);

    //check if returned column is unit vector
    float sum = v_l_col.norm();
    if (fabs(sum-1.0) > 1.0e-8){
      cout << "\n SVD returned non-unit vector.." << endl;
      return false;
    }

    //debug
    cout << "\nSolved!\n" << v_l_col << endl;
    
    //Set ground coefficients
    ground_coeffs.at(0) = v_l_col(0);
    ground_coeffs.at(1) = v_l_col(1);
    ground_coeffs.at(2) = v_l_col(2);
    ground_coeffs.at(3) = v_l_col(3);

    //debug - check plane fit
    double diff_max=0.0;
    for(int pt_i=0; pt_i<pt_mat.rows(); pt_i++){
      Eigen::Vector4f pt = pt_mat.row(pt_i);

      float temp_dist = pt.dot(v_l_col);

      //debug
      cout << "\nDot:" << temp_dist << endl;

      if (fabs(temp_dist)>diff_max)
  	diff_max = fabs(temp_dist);
    }
    cout << "\nMax difference= " << diff_max*1000.0 << "mm" << endl;
      

  }
  return true;
}


void GroundPlane::writeFile(string fileName)
{
  ofstream plane_file(fileName.c_str());
  if (plane_file.is_open())
    {
      plane_file << "Ground Coefficients:-" << endl;
      for (int i=0; i < ground_coeffs.size(); i++){
	plane_file << fixed << setprecision(10) << ground_coeffs.at(i);
	if ((i+1) != ground_coeffs.size())
	  plane_file << endl;
      }
      plane_file.close();
    }   

  return;
}

GroundPlane::GroundPlane(string fileName)
{
  ground_coeffs.resize(4);
  ifstream plane_file(fileName.c_str());
  if (plane_file.is_open())
    {
      //ignore first line
      string line;
      getline(plane_file, line);
      int i = 0;
      for(; getline(plane_file, line); ){
	istringstream in(line);
	in >> ground_coeffs.at(i);
	i++;
      }
      plane_file.close();
    }  
  else{cout << "\nCan not open file" << endl;}
  //debug
  cout << "\nPlane Parameters:-\n" << "a: " << ground_coeffs.at(0) 
       << "\t b: " << ground_coeffs.at(1) 
       << "\t c: " << ground_coeffs.at(2) 
       << "\t d: " << ground_coeffs.at(3) << endl;

  return;
}


void GroundPlane::visualizePlane(const PointCloudT::Ptr& cloud, 
				 double inter_thresh /*=0.01*/)
{
  cv::Scalar intersection_col = cv::Scalar(0,0, 255);
  cv::Mat result;
  int pc_rows = cloud->height;
  int pc_cols = cloud->width;
  double* coeffs_arr = &ground_coeffs[0];

  cv::Mat ground_mat = cv::Mat(1, ground_coeffs.size(), CV_64F, coeffs_arr);
  result.create(pc_rows, pc_cols, CV_8UC3);
      
  for (int r=0; r<pc_rows; r++){
    cv::Vec3b* res_i = result.ptr<cv::Vec3b>(r);
    for (int c=0; c<pc_cols; c++){
      PointT point = cloud->at(c,r);
      //check if plane intersection
      cv::Mat pt_mat = (cv::Mat_<double>(4,1) << 
			point.x, point.y, point.z, 1.0);
      double dot = (cv::Mat(ground_mat * pt_mat)).at<double>(0,0);
      //NaNs not considered for plane fit
      if (dot < inter_thresh && !isnan(point.z)){
	res_i[c][0] = intersection_col[0];
	res_i[c][1] = intersection_col[1];
	res_i[c][2] = intersection_col[2];
      }
      else{
	Eigen::Vector3i rgb = point.getRGBVector3i();
	res_i[c][0] = rgb[2];
	res_i[c][1] = rgb[1];
	res_i[c][2] = rgb[0];
      }
    }
  }
  //visualize
  imshow("Plane Cutting", result);
  cv::waitKey(0);
  cv::destroyWindow("Plane Cutting");
  return;
}

void GroundPlane::planePtsMask(const PointCloudT::Ptr& cloud, cv::Mat& mask_im, 
			       double inter_thresh/*=0.02*/)
{
  int pc_rows = cloud->height;
  int pc_cols = cloud->width;
  double* coeffs_arr = &ground_coeffs[0];

  cv::Mat ground_mat = cv::Mat(1, ground_coeffs.size(), CV_64F, coeffs_arr);

  //incase mask not correctly allocated
  if (mask_im.size()!=cv::Size(pc_rows, pc_cols) || mask_im.depth()!=CV_8UC1)
    {mask_im = cv::Mat::ones(pc_rows, pc_cols, CV_8UC1);}
  else
    {mask_im = cv::Scalar(1);}
      
  for (int r=0; r<pc_rows; r++){
    unsigned char* res_i = mask_im.ptr<unsigned char>(r);
    for (int c=0; c<pc_cols; c++){
      PointT point = cloud->at(c,r);
      //check if plane intersection
      cv::Mat pt_mat = (cv::Mat_<double>(4,1) << 
			point.x, point.y, point.z, 1.0);
      double dot = (cv::Mat(ground_mat * pt_mat)).at<double>(0,0);
      //NaNs not considered for plane fit
      if (dot <= inter_thresh && !isnan(point.z)){
	res_i[c] = (unsigned char) 0;
      }
    }
  }

  //debug-visualize-mask
  // cv::Mat disp_im;
  // mask_im.convertTo(disp_im, CV_32F);
  // imshow("Plane Mask", disp_im);
  // cv::waitKey(0);
  // cv::destroyWindow("Plane Mask");
  return;
}

// taken from PCL - tutorial
void GroundPlane::pcProject(const PointCloudT::Ptr& cloud, PointCloudT::Ptr& cloud_projected)
{
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  coefficients->values.resize (4);
  coefficients->values[0] = ground_coeffs.at(0);
  coefficients->values[1] = ground_coeffs.at(1);
  coefficients->values[2] = ground_coeffs.at(2);
  coefficients->values[3] = ground_coeffs.at(3);

  // Create the filtering object
  pcl::ProjectInliers<PointT> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (cloud);
  proj.setModelCoefficients(coefficients);
  proj.filter (*cloud_projected);
}
