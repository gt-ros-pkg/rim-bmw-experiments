#include<bmw_percep/groundPlane.hpp>
#include<iostream>
#include<pcl/io/openni_grabber.h>
#include<opencv2/opencv.hpp>
#include<bmw_percep/pcl_cv_utils.hpp>
#include<bmw_percep/particleFilter.hpp>

#include<geometry_msgs/PoseStamped.h>
//ros-includes
#include<ros/ros.h>
/**
   Sample program for implementing the complete chain of events for people tracking
**/

typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PointCloudX;

boost::mutex cloud_mutex;
enum { COLS=640, ROWS=480};

//Callback -- grabbing pointclouds from OpenNI
void cloud_cb_ (const PointCloudX::ConstPtr &callback_cloud, 
		PointCloudX::Ptr& cloud,
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
	       cv::Point3f &mean_blob, const PointCloudX::Ptr& cloud,
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
  ros::Publisher pub_pos = nh.advertise<geometry_msgs::PoseStamped> ("/human/position", 1);
  ros::Publisher pub_vel = nh.advertise<geometry_msgs::PoseStamped> ("/human/velocity", 1);

  //messages -- initialize
  geometry_msgs::PoseStamped pos_msg, vel_msg;
  pos_msg.header.frame_id = "base_link";
  vel_msg.header.frame_id = "base_link";
  pos_msg.header.seq = 0; vel_msg.header.seq = 0;
  pos_msg.pose.position.z=0; vel_msg.pose.position.z=0; 
  pos_msg.pose.orientation.x=0; pos_msg.pose.orientation.y=0; 
  pos_msg.pose.orientation.z=0; pos_msg.pose.orientation.z=0;
  vel_msg.pose.orientation = pos_msg.pose.orientation;

  geometry_msgs::PoseStamped noPerson_pos_msg, noPerson_vel_msg;
  noPerson_pos_msg = pos_msg;
  noPerson_pos_msg.pose.position.x = nan();
  noPerson_pos_msg.pose.position.y = nan();
  noPerson_pos_msg = noPerson_vel_msg;
  
  //Various objects
  PointCloudX::Ptr cloud (new PointCloudX);
  PointCloudX::Ptr cloud2 (new PointCloudX);

  int bg_history = 50;
  double bg_varThresh=0.03;
  bool bg_detectShadow=false;
  double init_learn=-1.0; // learning rate for initialization
  //TODO: fix learning rates
  double general_learn= 0.0;//0.001; // after initial period
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
  int min_human_pixels = 1500; // min. pixels for blob to be human

  char win_key=0;

  cv::Mat depth_im, valid_depth, foreMask, back_im, disp_im;
  bool bg_init = false; // has background been initialized
  //TODO: initialize these mats

  // Read Kinect live stream:
  bool new_cloud_available_flag = false;
  pcl::Grabber* interface = new pcl::OpenNIGrabber();
  boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> f =
    boost::bind (&cloud_cb_, _1, cloud, &new_cloud_available_flag);
  interface->registerCallback (f);
  interface->start ();

  // Wait for the first frame:
  while(!new_cloud_available_flag) 
    boost::this_thread::sleep(boost::posix_time::milliseconds(1));
  //new_cloud_available_flag = true;

  int n_frames=0;
  
  ros::Time begin, end;
  
  cout << "\nInitializing background ..." << endl;

      cv::Mat translate = (cv::Mat_<double>(3,1) << -1.1073859, -0.73154575, -2.3490002);
      double z_rot_cos = 0.3764;
      double z_rot_sin = -sqrt(1-pow(z_rot_cos,2));
      cv::Mat rot_mat = (cv::Mat_<double>(3,3)<<
			 z_rot_cos, -z_rot_sin, 0, 
			 z_rot_sin, z_rot_cos,  0,
			 0,         0,          1 );

  
  while(ros::ok()){

    if (new_cloud_available_flag && cloud_mutex.try_lock()){
      new_cloud_available_flag = false;
      n_frames++;

      // chain of events start
   
      // get image, depth-map, valid mask
      cv_utils::pc_to_depth(cloud, depth_im, valid_depth);
      
      //back-ground initialize
      if (!bg_init){
	cv::imshow("Init", depth_im);
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

      begin = ros::Time::now();
          
      //debug
      // cout << "Foreground masking begins? " << endl;
      
      //get foreground mask without learning from image
      //cvBg.operator()(rgb_im, foreMask, 0);
      cvBg.operator()(depth_im, foreMask, general_learn);
    
      //debug
      //cv::Mat dep_norm;
      //cv::normalize (depth_im, dep_norm);
      //cv::imshow("Depth", dep_norm);

      //apply depth-mask
      // foreMask.copyTo(foreMask, valid_depth);
      // cv::imshow("fore", foreMask);
      // cv::waitKey(15);
      // cloud_mutex.unlock();
      // continue;

      
      //debug
      //cout << "Foregrounding ..." << endl;

      //ground-subtract -- not required
      //ground_obj.planePtsMask(cloud, ground_mask, 0.03);
      //foreMask.copyTo(foreMask, ground_mask);

      // cv::imshow("Fore Image", foreMask);
      // cv::waitKey(10);
      // cloud_mutex.unlock();
      // continue;

      //foreMask = 255 * foreMask;

      //translate then rotate point cloud
      // double translate[] = {-1.1073859, -0.73154575, -2.3490002};
      // double z_rot_cos = 0.3764;
      // double z_rot_sin = sqrt(1-pow(z_rot_cos,2));
      // double t1[] = 
      // t1 << 
      // 	1, 0, 0, translate[0],
      // 	0, 1, 0, translate[1],
      // 	0, 0, 1, translate[2],
      // 	0, 0, 0, 1;
      
      // cout << t1<< endl;
      // pcl::PointIndices ::Ptr unmasked_indi (new pcl::PointIndices());
      // unmasked_indi->indices.push_back(1);
      // //pcl::transformPointCloud(cloud, cloud2, &t1);
      
      //transform_cloud(cloud, translate)

      //find human component in foreground
      vector< vector <cv::Point2i> > fore_blobs;
      cv::Point3f blob_mean;
      bool found_human = get_blobs(foreMask, min_human_pixels, 
				   fore_blobs, blob_mean, cloud);
    
      // can't simply continue, as tracking requires stuff
      if (!found_human){
	//	cvBg.operator()(rgb_im, foreMask, general_learn); // no human, so learn all
	//cvBg.operator()(depth_im, foreMask, general_learn);
	cloud_mutex.unlock(); 
	continue;
      } // nothing else to do in this case
      
      //debug - draw contours
      depth_im.copyTo(disp_im);
      //cv::imshow("Fore Image", foreMask); 
      // cv::drawContours(disp_im, fore_blobs, -1, cv::Scalar(0, 127, 127));
      // cv::imshow("Human Contours", disp_im);
      // cv::waitKey(15);

      //project onto ground plane
      //rotate
      cv::Mat temp_pt = (cv::Mat_<double> (3,1) << blob_mean.x, blob_mean.y, blob_mean.z);
      cv::Mat new_pt = rot_mat * temp_pt;
      new_pt = new_pt + translate;

      cv::Point2f hum_pt; hum_pt.x=new_pt.at<double>(0,0); hum_pt.y=new_pt.at<double>(0,1);
      
      //debug
      // cout << "3d Point:" << blob_mean << endl; 
      // cout << "Temp point: " << temp_pt << endl;
      // cout << "Human Point " << hum_pt << endl;
      // cout << "New pt: " << new_pt << endl;
      //return 0;
      //particle tracking

      //add in non-human as background
      //cvBg.operator()(rgb_im, foreMask, general_learn); // no human, so learn all
      //cvBg.operator()(depth_im, foreMask, general_learn); // no human, so learn all

      //publish points
      
      ros::Time cur_time = ros::Time::now();
      pos_msg.header.seq++;
      vel_msg.header.seq++;
      pos_msg.header.stamp = cur_time;
      vel_msg.header.stamp = cur_time;

      pos_msg.pose.position.x = hum_pt.x; 
      pos_msg.pose.position.y = hum_pt.y; 
      pos_msg.pose.position.z = 0; // 2D points 

      vel_msg.pose.position.x = hum_pt.x;
      vel_msg.pose.position.y = hum_pt.y; 
      vel_msg.pose.position.z = 0; // 2D points 
      
      //velocity computation
      if (isnan(prev_pos_msg.pose.position.x)){
	pub_pos.publish(noPerson_pos_msg);
	pub_vel.publish(noPerson_vel_msg);
      }
      else{
	//velocity computation

	pub_pos.publish(pos_msg);
	pub_vel.publish(vel_msg);
      }
      end = ros::Time::now();
      //debug
      //cout << "Iteration time: " << end-begin << endl;
      //debug
      //cout << "To be published "<< hum_pt << endl;

      cloud_mutex.unlock (); // let go of current cloud
    }
  }

  return 0; // successfully exit
}

//get blobs from a foreground image
//TODO: use the depth not color/project on the ground
bool get_blobs(cv::Mat& fore, int min_pixels,
	      vector < vector<cv::Point2i> > &detect_blobs,
	       cv::Point3f &mean_blob, const PointCloudX::Ptr& cloud,
	       int ERODE_SIZE/*=5*/, int DILATE_SIZE/*=5*/, bool debug_mode/*=false*/)
{
  int max_blob_id;
  float mean_blob_x, mean_blob_y, mean_blob_z;

  //Morph Elements
  cv::Mat struct_erode1 = cv::Mat::ones(8,2, CV_8U);
  cv::Mat struct_erode = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ERODE_SIZE, ERODE_SIZE));
  cv::Mat struct_dilate = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  //morph
  cv::erode(fore, fore, struct_erode1);
  cv::erode(fore, fore, struct_erode);
  cv::dilate(fore, fore, struct_dilate);

  //debug
  // cv::imshow("Fore",fore);
  // cv::waitKey(10);

  
  //binarize
  //TODO: check if this is the correct depth
  fore.convertTo(fore, CV_32F);
  cv::threshold(fore, fore, 0., 1, cv::THRESH_BINARY);

  //debug
  // cv::imshow("Fore2",fore);
  // cv::waitKey(10);

    
  if (debug_mode)
    {
      cv::Mat disp_fore;
      fore.convertTo(disp_fore, -1, 255);
      cv::namedWindow("Fore",1);
      cv::imshow("Fore", disp_fore);
      char c = cv::waitKey(5);
    }
  
  //find-blobs
  int max_blob_size = cv_utils::find_blobs(fore, detect_blobs, max_blob_id);

  //debug
  //cout << "Max blob : " << max_blob_size << "  ID: "<< max_blob_id << "  Actual Size : " << detect_blobs[max_blob_id].size() << endl;

  //check the size of the max blob
  if (max_blob_size>min_pixels)
    {  
      //blob-mean	
      mean_blob_x = 0; mean_blob_y = 0; mean_blob_z=0;
      for (int i=0; i<max_blob_size; i++)
  	{
	  //cout << "I cause problem: " << i << endl;
	  PointX point = cloud->at(detect_blobs[max_blob_id][i].x, detect_blobs[max_blob_id][i].y);
	  if (isnan(point.x) || isnan(point.y) || isnan(point.z))
	    {
	      max_blob_size--;
	      continue;
	    }
	  mean_blob_x += point.x;
	  mean_blob_y += point.y;
	  mean_blob_z += point.z;
	}
      mean_blob_x /= (float) max_blob_size;
      mean_blob_y /= (float) max_blob_size;
      mean_blob_z /= (float) max_blob_size;
      
      //setting what I care about
      //cout << "Got here?" << endl;
      mean_blob.x=mean_blob_x; mean_blob.y=mean_blob_y; mean_blob.z=mean_blob_z;
      
      return true;
    }
  
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
