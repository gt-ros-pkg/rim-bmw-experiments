#include<bmw_percep/groundPlane.hpp>
#include<iostream>
#include<pcl/io/openni_grabber.h>
#include<opencv2/opencv.hpp>
#include<bmw_percep/pcl_cv_utils.hpp>
#include<bmw_percep/particleFilter.hpp>

//ros-includes
#include<ros/ros.h>
/**
   Sample program for implementing the complete chain of events for people tracking
**/

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

boost::mutex cloud_mutex;
enum { COLS=640, ROWS=480};

//Callback -- grabbing pointclouds from OpenNI
void cloud_cb_ (const PointCloudT::ConstPtr &callback_cloud, 
		PointCloudT::Ptr& cloud,
		bool* new_cloud_available_flag)
{
  cloud_mutex.lock ();    // for not overwriting the point cloud from
			  // another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}

bool get_blobs(cv::Mat& fore, int min_pixels,
	      vector < vector<cv::Point2i> > &detect_blobs,
	       int ERODE_SIZE=5, int DILATE_SIZE=5, bool debug_mode=false);

//get blobs from a foreground image
 bool get_blobs_depth(cv::Mat& fore, cv::Mat depth, int min_pixels,
		      vector < vector<cv::Point2i> > &detect_blobs, 
		      double depth_delta=0.05,
		      int ERODE_SIZE=5, int DILATE_SIZE=5, 
		      bool debug_mode=false);


  int main(int argc, char** argv)
{

  //ros
  ros::init(argc, argv, "sample_tracker");
  ros::NodeHandle nh;

  //Various objects
  PointCloudT::Ptr cloud (new PointCloudT);
  int bg_history = 50;
  double bg_varThresh=0.03;
  bool bg_detectShadow=false;
  double init_learn=-1.0; // learning rate for initialization
  //TODO: fix learning rates
  double general_learn= 0.5; // after initial period
  double after_img=0.3;
  cv::BackgroundSubtractorMOG2 cvBg(bg_history, bg_varThresh, bg_detectShadow);
  cvBg.setInt("nmixtures", 2);

  particleFilter2D filter_human;
  
  //ground-plane
  //read ground plane parameters from file
  string fileName = "data/ground_coeffs.txt";
  GroundPlane ground_obj(fileName);
  cv::Mat ground_mask;

  //Tracker params
  int min_human_pixels = 50; // min. pixels for blob to be human

  char win_key=0;

  cv::Mat rgb_im, depth_im, valid_depth, foreMask, back_im, disp_im;
  bool bg_init = false; // has background been initialized
  //TODO: initialize these mats

  // Read Kinect live stream:
  bool new_cloud_available_flag = false;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =
    boost::bind (&cloud_cb_, _1, cloud, &new_cloud_available_flag);
  interface->registerCallback (f);
  interface->start ();

  // Wait for the first frame:
  while(!new_cloud_available_flag) 
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  //new_cloud_available_flag = true;

  int n_frames=0;

  cout << "\nInitializing background ..." << endl;
  
  while(ros::ok()){

    if (new_cloud_available_flag && cloud_mutex.try_lock()){
      new_cloud_available_flag = false;
      n_frames++;

      // chain of events start
   
      // get image, depth-map, valid mask
      cv_utils::pc_to_img(cloud, rgb_im, depth_im, valid_depth);

      
      //back-ground initialize
      if (!bg_init){
	cv::imshow("Init", rgb_im);
	//cvBg.operator()(rgb_im, foreMask, init_learn);
	cvBg.operator()(depth_im, foreMask, init_learn);
	win_key = cv::waitKey(15);

	/* Can't change history condition in OpenCV currently
	//if more than history frames seen for initialization
	if (bg_history < n_frames){
	//set new history limit
	bg_history = n_frames;
	//cvBg.setHistory(bg_history);
	}
	*/

	//Quit or Not
	if (win_key!=27){
	  if (n_frames == bg_history)
	    cout << "\n Background initialized - press 'Esc' to exit." << endl;
	  // no further processing required
	  cloud_mutex.unlock();
	  continue;
	}
	else{
	  if (n_frames < bg_history){
	    cout << "\nHistory frames not reached... Continue recording background." 
		 << endl;
	    cloud_mutex.unlock();
	    continue;
	  }
	  bg_init = true;
	  cout <<  "Background Initialized!" << endl; 
	  //to the next frame we go..

	  //debug
	  //cvBg.getBackgroundImage(back_im);
	  //cv::imshow("background image", back_im);
	  //cv::waitKey(0);
	  
	  //debug
	  //cout << "\nTime to destroy .. " << endl;
	  cv::destroyWindow("Init");
	  //cv::destroyAllWindows();
	  //debug
	  //cout << "\n DESTROYED.. " << endl;

	  cloud_mutex.unlock();
	  continue;
	}
      }
          
      //debug
      // cout << "Foreground masking begins? " << endl;
      
      //get foreground mask without learning from image
      //cvBg.operator()(rgb_im, foreMask, 0);
      cvBg.operator()(depth_im, foreMask, general_learn);
    
      //debug
      //cv::Mat dep_norm;
      //cv::normalize (depth_im, dep_norm);
      //cv::imshow("Depth", dep_norm);
      cv::imshow("Fore Image", foreMask);
      cv::waitKey(10);
      cloud_mutex.unlock();
      continue;

      //apply depth-mask
      foreMask.copyTo(foreMask, foreMask);
      
      //debug
      //cout << "Foregrounding ..." << endl;

      //ground-subtract -- not required
      ground_obj.planePtsMask(cloud, ground_mask);
      foreMask.copyTo(foreMask, ground_mask);
      //foreMask = 255 * foreMask;

      //find human component in foreground
      vector< vector <cv::Point2i> > fore_blobs;
      bool found_human = get_blobs(foreMask, min_human_pixels,
			      fore_blobs);
    
      // can't simply continue, as tracking requires stuff
      if (!found_human){
	//	cvBg.operator()(rgb_im, foreMask, general_learn); // no human, so learn all
	cvBg.operator()(depth_im, foreMask, 0);
	cloud_mutex.unlock(); 
	continue;
      } // nothing else to do in this case
      
      //debug - draw contours
      rgb_im.copyTo(disp_im);
      cv::drawContours(disp_im, fore_blobs, -1, cv::Scalar(0, 127, 127));
      //cv::imshow("Human Contours", disp_im);
      cv::waitKey(15);

      //project on ground plane

      //particle tracking

      //add in non-human as background
      //cvBg.operator()(rgb_im, foreMask, general_learn); // no human, so learn all
      cvBg.operator()(depth_im, foreMask, general_learn); // no human, so learn all
      
      cloud_mutex.unlock (); // let go of current cloud
    }
  }

  return 0; // successfully exit
}

//get blobs from a foreground image
//TODO: use the depth not color/project on the ground
bool get_blobs(cv::Mat& fore, int min_pixels,
	      vector < vector<cv::Point2i> > &detect_blobs,
	       int ERODE_SIZE/*=5*/, int DILATE_SIZE/*=5*/, bool debug_mode/*=false*/)
{
  int max_blob_id, max_blob_area;
  double mean_blob_x, mean_blob_y;

  //Morph Elements
  cv::Mat struct_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ERODE_SIZE, ERODE_SIZE));
  cv::Mat struct_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  
  //morph
  cv::erode(fore, fore, struct_erode);
  cv::dilate(fore, fore, struct_dilate);
  
  //binarize
  //TODO: check if this is the correct depth
  cv::threshold(fore, fore, 0., 1, cv::THRESH_BINARY);
    
  if (debug_mode)
    {
      cv::Mat disp_fore;
      fore.convertTo(disp_fore, -1, 255);
      cv::namedWindow("Fore",1);
      cv::imshow("Fore", disp_fore);
      char c = cv::waitKey(5);
    }
  
  //find-blobs
  int max_blob_size = cv_utils::find_blobs(fore, detect_blobs);

  //check the size of the max blob
  if (max_blob_size>min_pixels){return true;}
  //   {  
  //     //blob-mean	
  //     mean_blob_x = 0; mean_blob_y = 0;
  //     for (int i=0; i<max_blob_area; i++)
  // 	{
  // 	  mean_blob_x += detect_blobs[max_blob_id][i].x;
  // 	  mean_blob_y += detect_blobs[max_blob_id][i].y;
  // 	}
  //     mean_blob_x /= max_blob_area;
  //     mean_blob_y /= max_blob_area;
      
  //     //setting what I care about
  //     mean_b_x = mean_blob_x;
  //     mean_b_y = mean_blob_y;
      
  //     //get grid position
  //     return compute_grid_pos(mean_blob_x, mean_blob_y, fore.rows, fore.cols);
    // }
  
  //if no object big enoungh
  else{return false;}
 }


  //get blobs from a foreground image
 bool get_blobs_depth(cv::Mat& fore, cv::Mat depth, int min_pixels,
		      vector < vector<cv::Point2i> > &detect_blobs, 
		      double depth_delta/*=0.05*/,
		      int ERODE_SIZE/*=5*/, int DILATE_SIZE/*=5*/, 
		      bool debug_mode/*=false*/)
{
  int max_blob_id, max_blob_area;
  double mean_blob_x, mean_blob_y;

  //Morph Elements
  cv::Mat struct_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ERODE_SIZE, ERODE_SIZE));
  cv::Mat struct_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  
  //morph
  cv::erode(fore, fore, struct_erode);
  cv::dilate(fore, fore, struct_dilate);
  
  //binarize
  if (fore.depth()!=CV_32FC1){fore.convertTo(fore, CV_32FC1);}
  cv::threshold(fore, fore, 0., 1, cv::THRESH_BINARY);
  cv::Mat depth_mask; fore.convertTo(depth_mask, CV_8UC1, 255);
    
  if (debug_mode)
    {
      cv::Mat disp_fore;
      fore.convertTo(disp_fore, -1, 255);
      cv::namedWindow("Fore",1);
      cv::imshow("Fore", disp_fore);
      char c = cv::waitKey(5);
    }
  
  //find-blobs
  int max_blob_size = cv_utils::find_blobs_depth(depth_mask, depth, detect_blobs, depth_delta);

  //check the size of the max blob
  if (max_blob_size>min_pixels){return true;}
  //   {  
  //     //blob-mean	
  //     mean_blob_x = 0; mean_blob_y = 0;
  //     for (int i=0; i<max_blob_area; i++)
  // 	{
  // 	  mean_blob_x += detect_blobs[max_blob_id][i].x;
  // 	  mean_blob_y += detect_blobs[max_blob_id][i].y;
  // 	}
  //     mean_blob_x /= max_blob_area;
  //     mean_blob_y /= max_blob_area;
      
  //     //setting what I care about
  //     mean_b_x = mean_blob_x;
  //     mean_b_y = mean_blob_y;
      
  //     //get grid position
  //     return compute_grid_pos(mean_blob_x, mean_blob_y, fore.rows, fore.cols);
    // }
  
  //if no object big enoungh
  else{return false;}
}
