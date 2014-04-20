#include<bmw_percep/groundPlane.hpp>
#include<iostream>
#include<pcl/io/openni_grabber.h>
#include<opencv2/opencv.hpp>
#include<bmw_percep/pcl_cv_utils.hpp>
#include<bmw_percep/particleFilter.hpp>

//ros-includes
#include<ros/ros.h>
#include<visualization_msgs/MarkerArray.h>
#include<visualization_msgs/Marker.h>
#include<std_msgs/UInt8.h>
#include<std_msgs/Bool.h>
#include<geometry_msgs/PoseStamped.h>

/**
   Sample program for implementing the complete chain of events for people tracking
**/

typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointX> PointCloudX;

// Globals
visualization_msgs::MarkerArray mark_arr;
int frame_rate=30;
double max_filt_vel = 15.0/static_cast<double>(frame_rate), 
  max_filt_acc = 5.0/static_cast<double>(frame_rate); // per frame

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

void filter_estimate_avg(const cv::Point2f prev_pos, const cv::Point2f prev_vel, 
		     cv::Point2f &cur_pos, cv::Point2f &cur_vel);

void filter_estimate(const cv::Point2f prev_pos, const cv::Point2f prev_vel, 
		     cv::Point2f &cur_pos, cv::Point2f &cur_vel);


void box_filter(const PointCloudX::Ptr& cloud, cv::Point3f end1, cv::Point3f end2,
		cv::Mat& mask);

void box_filter_inverse(const PointCloudX::Ptr& cloud, cv::Point3f end1, cv::Point3f end2,
		cv::Mat& mask);

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

int publish_human_markers(ros::Publisher viz_pub, geometry_msgs::PoseStamped pos, geometry_msgs::PoseStamped vel, string hum_frame);

int main(int argc, char** argv)
{
  //ros
  ros::init(argc, argv, "sample_tracker");
  ros::NodeHandle nh;
  ros::Publisher pub_pos = nh.advertise<geometry_msgs::PoseStamped> ("/human/position", 1);
  ros::Publisher pub_vel = nh.advertise<geometry_msgs::PoseStamped> ("/human/velocity", 1);
  ros::Publisher pub_viz = nh.advertise<visualization_msgs::MarkerArray> ("/human/visuals", 1);
  ros::Publisher pub_robo_state = nh.advertise<std_msgs::UInt8> ("/human/robot", 1);

  ros::Publisher av_stop = nh.advertise<std_msgs::Bool> ("/person_avoid/stop", 1);
  ros::Publisher a_clear = nh.advertise<std_msgs::Bool> ("person_avoid/all_clear", 1);
  ros::Publisher mv_front = nh.advertise<std_msgs::Bool> ("/person_avoid/move_front", 1);


  //messages -- initialize
  string hum_frame = "base_link";
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
  noPerson_pos_msg.pose.position.x = numeric_limits<double>::quiet_NaN();
  noPerson_pos_msg.pose.position.y = numeric_limits<double>::quiet_NaN();
  noPerson_vel_msg = noPerson_pos_msg;
  
  //Various objects
  PointCloudX::Ptr cloud (new PointCloudX);
  PointCloudX::Ptr cloud2 (new PointCloudX);
  PointCloudX::Ptr cloud_in (new PointCloudX);

  int bg_history = 5;//0;
  double bg_varThresh=0.03;
  bool bg_detectShadow=false;
  double init_learn=-1.0; // learning rate for initialization
  //TODO: fix learning rates
  double general_learn= 0.0;//0.001; // after initial period
  double after_img=0.3;
  cv::BackgroundSubtractorMOG2 cvBg(bg_history, bg_varThresh, bg_detectShadow);
  cvBg.setInt("nmixtures", 2);

  particleFilter2D filter_human;
  bool currently_filtering=false; // to check whether reinitialization reqd

  //ground-plane
  //read ground plane parameters from file
  string fileName = "data/ground_coeffs.txt";
  GroundPlane ground_obj(fileName);
  cv::Mat ground_mask;

  //Tracker params
  int min_human_pixels = 6000; // min. pixels for blob to be human

  char win_key=0;

  cv::Mat depth_im, valid_depth, foreMask, back_im, disp_im, box_mask, box_mask_inliers;

  //  cv::Point3f box_min = cv::Point3f(0.9, 0.18, 1.9);
  //  cv::Point3f box_max = cv::Point3f(1.24, 0.44, 3.06);

  cv::Point3f box_min = cv::Point3f(0.0, -0.5, 1.0);//(1.24, 0.55, 3.06);
  cv::Point3f box_max = cv::Point3f(1.2, 1.0, 3.0);//(0.8, 0.05, 1.9);

  cv::Point3f boxin_min, boxin_max;

  // If params for inlier box specified - use them
  if (argc == 7){
    boxin_min = 
      cv::Point3f(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    boxin_max = 
      cv::Point3f(atof(argv[4]), atof(argv[5]), atof(argv[6]));
  }
  // else previously stored params
  else {
    boxin_min = cv::Point3f(-1.0, -2.0, -10.0);
    boxin_max = cv::Point3f(2.0, 0.5, 10.0);
  }
  
  bool bg_init = false; // has background been initialized

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

  int n_frames=0;
  

  //vars to help profile
  ros::Time begin, end;
  
  cout << "\nInitializing background ..." << endl;

  //Transform into the ground plane frame of reference
  cv::Mat translate = (cv::Mat_<double>(3,1) << -1.1073859, -0.73154575, -2.3490002);
  cv::Mat rotate_mat;
  double z_rot_cos = 0.3764;
  double z_rot_sin = -sqrt(1-pow(z_rot_cos,2));
  cv::Mat rot_mat1 = (cv::Mat_<double>(3,3)<<
		      z_rot_cos, -z_rot_sin, 0, 
		      z_rot_sin, z_rot_cos,  0,
		      0,         0,          1 );

  double y_rot_cos = 0.05378;
  double y_rot_sin = sqrt(1-pow(y_rot_cos,2));
  cv::Mat rot_mat2 = (cv::Mat_<double>(3,3)<<
		      y_rot_cos,  0,          y_rot_sin,
		      0,          1,          0,
		      -y_rot_sin,  0,          y_rot_cos);
  rotate_mat = rot_mat2 * rot_mat1;

  while(ros::ok()){
    
    if (new_cloud_available_flag && cloud_mutex.try_lock()){

      //from second frame onwards
      if(n_frames>0){
	// check frames
	pos_msg.header.frame_id = hum_frame;
	vel_msg.header.frame_id = hum_frame;
	pub_pos.publish(pos_msg);
	pub_vel.publish(vel_msg);
	int robo_state = publish_human_markers(pub_viz, pos_msg, 
					       vel_msg, hum_frame);
	
        std_msgs::Bool true_msg;
        std_msgs::Bool false_msg;
        true_msg.data = true;
        false_msg.data = false;
	switch(robo_state)
	  {

	  case 1: 
	    a_clear.publish(true_msg);
	    mv_front.publish(true_msg);
	    av_stop.publish(false_msg);
	    break;
	  case 2: 
	    a_clear.publish(false_msg);
	    mv_front.publish(false_msg);
	    av_stop.publish(false_msg);
	    break;
	  case 3: 
	    a_clear.publish(false_msg);
	    mv_front.publish(false_msg);
	    av_stop.publish(true_msg);
	    break;
	  }
	//publish to robot
	std_msgs::UInt8 pub_rob;
	pub_rob.data = robo_state;
	pub_robo_state.publish(pub_rob);

    }

      new_cloud_available_flag = false;
      n_frames++;

      // Start Image processing pipeline
   
      // get image, depth-map, valid mask from extracted point cloud
      cv_utils::pc_to_depth(cloud, depth_im, valid_depth);
      
      //back-ground initialize -- for the first specified number of frames
      //TODO: from recorded data
      if (!bg_init){

	cvBg.operator()(depth_im, foreMask, init_learn);

	if (n_frames >= bg_history){
	  cout << 
	    "\n ***** Background initialized - Start actual processing" 
	       << endl;
	  // no further processing required for this frame
	bg_init = true;
	 
	pos_msg = noPerson_pos_msg;
	vel_msg = noPerson_vel_msg;

	cloud_mutex.unlock();
	continue;
	} 
      }

      begin = ros::Time::now();
      
      // TODO: learn newer backgrounds as new images are coming in

      // Get foreground mask without learning from image
      cvBg.operator()(depth_im, foreMask, general_learn);
      // filter out the 'robot box'
      box_filter(cloud, box_min, box_max, box_mask);
      // filter out points not in the inlier box
      box_filter_inverse(cloud, boxin_min, boxin_max, box_mask_inliers);
      // logical and of the two masks from the two boxes
      cv::bitwise_and(box_mask, box_mask_inliers, box_mask);
      
      // Apply the box mask to the foreground mask 
      foreMask.copyTo(foreMask, box_mask);

      //debug
      // imshow("Mask", box_mask);
      // cout << "showy boxy" << endl;
      // cv::Mat dep_show;
      // cv::normalize(depth_im, depth_im, 0.0, 1.0, cv::NORM_MINMAX);
      // //cv::applyColorMap (depth_im, dep_show, cv::COLORMAP_AUTUMN);
      // cv::imshow("Unfiltered", depth_im);
      // depth_im.copyTo(dep_show, box_mask);      
      // cv::imshow("Filtered", dep_show);
      // cv::waitKey(10);
      //cloud_mutex.unlock();
      //continue;
      
      //find human component in foreground
      vector< vector <cv::Point2i> > fore_blobs;
      cv::Point3f blob_mean;
      bool found_human = get_blobs(foreMask, min_human_pixels, 
				   fore_blobs, blob_mean, cloud);
    
      // can't simply continue, as tracking requires stuff
      if (!found_human){
	//cvBg.operator()(depth_im, foreMask, general_learn);
	pos_msg = noPerson_pos_msg;
	vel_msg = noPerson_vel_msg;
	cloud_mutex.unlock(); 
	continue;
      } // nothing else to do in this case
      
      //debug - draw contours
      //depth_im.copyTo(disp_im);
      //cv::imshow("Fore Image", foreMask); 
      // cv::drawContours(disp_im, fore_blobs, -1, cv::Scalar(0, 127, 127));
      // cv::imshow("Human Contours", disp_im);
      // cv::waitKey(15);

      // Transform to the 2D ground plane
      // TODO: check if the transformation is correct
      cv::Mat temp_pt = (cv::Mat_<double> (3,1) 
			 << blob_mean.x, blob_mean.y, blob_mean.z);
      cv::Mat new_pt = rotate_mat * temp_pt;

      //translate
      new_pt = new_pt + translate;
      
      cv::Point2f hum_pt; hum_pt.x=new_pt.at<double>(0,0); hum_pt.y = 
							     new_pt.at<double>(0,1);
      
      //particle tracking

      //compute pos, vel
      ros::Time cur_time = ros::Time::now();
      pos_msg.header.seq++;
      vel_msg.header.seq++;
      pos_msg.header.stamp = cur_time;
      vel_msg.header.stamp = cur_time;
      
      cv::Point2f prev_pos = cv::Point2f(pos_msg.pose.position.x, pos_msg.pose.position.y);
      cv::Point2f prev_vel = cv::Point2f(vel_msg.pose.position.x, vel_msg.pose.position.y);
      cv::Point2f cur_pos = hum_pt;
      cv::Point2f cur_vel;

      //filter_estimate(prev_pos, prev_vel, cur_pos, cur_vel);
      //filter_estimate_avg(prev_pos, prev_vel, cur_pos, cur_vel);
      if (!isnan(cur_pos.x)){      
	if (!isnan(prev_pos.x)){
	  if (currently_filtering){
	    filter_human.prop_part();
	    filter_human.estimate(hum_pt, cur_pos, cur_vel);
	    cout << "Observation "<< hum_pt << "-- Filtered: " << cur_pos << endl; 
	    cout << "Observed velocity"<< hum_pt-prev_pos << endl; 
	    //return 0;

	  }
	  else{
	    filter_human.reinitialize(prev_pos, hum_pt, 
				      (1.0/static_cast<double> (frame_rate)), 500,
				      5.0);
	    filter_human.estimate(hum_pt, cur_pos, cur_vel);
	    // cout << "Observation "<< hum_pt << "-- Filtered: " << cur_pos << endl; 
	    // cout << "Observed velocity"<< hum_pt-prev_pos << endl; 
	    // return 0;
	    currently_filtering = true;
	  }
	}
	else{
	  currently_filtering = false;
	  cur_vel.x = numeric_limits<double>::quiet_NaN();
	  cur_vel.y = numeric_limits<double>::quiet_NaN();
	}
      }
      else{
	currently_filtering = false;
	cur_vel.x = numeric_limits<double>::quiet_NaN();
	cur_vel.y = numeric_limits<double>::quiet_NaN();
      }

      pos_msg.pose.position.x = cur_pos.x; 
      pos_msg.pose.position.y = cur_pos.y; 
      pos_msg.pose.position.z = 0; // 2D points 

      vel_msg.pose.position.x = cur_vel.x; 
      vel_msg.pose.position.y = cur_vel.y; 
      vel_msg.pose.position.z = 0; // 2D points 

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

// filter out a box from cloud using mask
// assumes first end-pt has the lower values and second higher
void box_filter(const PointCloudX::Ptr& cloud, cv::Point3f end1, 
		cv::Point3f end2, cv::Mat& mask)
{
  //check if conversion possible
  if (cloud->isOrganized()){
    int pc_rows = cloud->height;
    int pc_cols = cloud->width;

    //TODO: Check if allocated Mat can be created
    if (!cloud->empty()){
      // allocate if neccessary
      if (mask.size()!=cv::Size(pc_rows, pc_cols) || mask.depth()!= CV_8UC1)
	mask = 255 * cv::Mat::ones(pc_rows, pc_cols, CV_8UC1);
      else
	mask = cv::Scalar(255);

      for (int r=0; r < pc_rows; r++){
	unsigned char* mask_r = mask.ptr<unsigned char> (r);
	for (int c=0; c < pc_cols; c++){
	  PointX point = cloud->at(c,r);
	  if (point.x>end1.x && point.y>end1.y && point.z>end1.z)
	    if (point.x<end2.x && point.y<end2.y && point.z<end2.z){
	      mask_r[c] = (unsigned char) 0;
	    }
	}
      }
    }
    else{
      cout << "\nCloud Empty!" << endl;
    }
  }
  else{
    cout << endl << "Cloud Unorganized.." << endl;
  }
  //debug
  //cv::imshow("Box", mask);
  
}

// filter out a box from cloud using mask
// assumes first end-pt has the lower values and second higher
void box_filter_inverse(const PointCloudX::Ptr& cloud, cv::Point3f end1, 
		cv::Point3f end2, cv::Mat& mask)
{
  //check if conversion possible
  if (cloud->isOrganized()){
    int pc_rows = cloud->height;
    int pc_cols = cloud->width;

    //TODO: Check if allocated Mat can be created
    if (!cloud->empty()){
      // allocate if neccessary
      if (mask.size()!=cv::Size(pc_rows, pc_cols) || mask.depth()!= CV_8UC1)
	mask = 0 * cv::Mat::ones(pc_rows, pc_cols, CV_8UC1);
      else
	mask = cv::Scalar(0);

      for (int r=0; r < pc_rows; r++){
	unsigned char* mask_r = mask.ptr<unsigned char> (r);
	for (int c=0; c < pc_cols; c++){
	  PointX point = cloud->at(c,r);
	  if (point.x>end1.x && point.y>end1.y && point.z>end1.z)
	    if (point.x<end2.x && point.y<end2.y && point.z<end2.z){
	      mask_r[c] = (unsigned char) 255;
	    }
	}
      }
    }
    else{
      cout << "\nCloud Empty!" << endl;
    }
  }
  else{
    cout << endl << "Cloud Unorganized.." << endl;
  }
  //debug
  //cv::imshow("Box", mask);
  
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

/**
   1- all allowed
   2- backward and front slow
   3- only backward
**/
int publish_human_markers( ros::Publisher viz_pub, geometry_msgs::PoseStamped pos, 
		      geometry_msgs::PoseStamped vel, string hum_frame)
{

  //debug
  const int all_allowed=1, front_slow=2, backward_only=3, robo_marker_id=4;
  int pubbed_state = all_allowed;

  visualization_msgs::Marker pos_marker, vel_marker1, vel_marker2, vel_marker3, 
    robo_marker;
  
  //only publish visual markers if velocity present
  if (isnan(vel.pose.position.x)){
    for (int i=0; i<mark_arr.markers.size(); i++){
      if(i!=robo_marker_id)
	mark_arr.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    //viz_pub.publish(mark_arr);
    pubbed_state =  all_allowed;
  }
  
 
  else{
    double delta_t = .5 * frame_rate; // .5 secs
  
    int n_vel_markers = 3;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    pos_marker.header.frame_id = hum_frame;
    pos_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    pos_marker.ns = "human";
    pos_marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    uint32_t cylinder = visualization_msgs::Marker::CYLINDER;
    pos_marker.type = cylinder;

    // Set the pos_marker action.  Options are ADD and DELETE
    pos_marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the pos_marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    pos_marker.pose.position.x = pos.pose.position.x;
    pos_marker.pose.position.y = pos.pose.position.y;
    pos_marker.pose.position.z = 0;
    pos_marker.pose.orientation.x = 0.0;
    pos_marker.pose.orientation.y = 0.0;
    pos_marker.pose.orientation.z = 0.0;
    pos_marker.pose.orientation.w = 1.0;
  
    // Set the scale of the pos_marker -- 1x1x1 here means 1m on a side
    pos_marker.scale.x = 0.3;
    pos_marker.scale.y = 0.3;
    pos_marker.scale.z = 2.0;
  
    // Set the color -- be sure to set alpha to something non-zero!
    pos_marker.color.r = 1.0f;
    pos_marker.color.g = 0.0f;
    pos_marker.color.b = 0.0f;
    pos_marker.color.a = 1.0;

    pos_marker.pose.position.z += pos_marker.scale.z/2;
  
    pos_marker.lifetime = ros::Duration();

    //velocity markers
    vel_marker1 = pos_marker;
    vel_marker1.id = 1;
    vel_marker1.pose.position.z = 0;

    vel_marker1.pose.position.x = pos.pose.position.x + delta_t * vel.pose.position.x;
    vel_marker1.pose.position.y = pos.pose.position.y + delta_t * vel.pose.position.y;

    vel_marker1.scale.x = 0.6;
    vel_marker1.scale.y = 0.6;
    vel_marker1.scale.z = 0.01;
    
    vel_marker1.color.a = 1.0f;
    vel_marker1.color.r = 0.0f;
    vel_marker1.color.g = 1.0f;

    mark_arr.markers.clear();

    mark_arr.markers.push_back(pos_marker);
    
    vel_marker2 = vel_marker1;
    vel_marker3 = vel_marker2;
    vel_marker2.id = 2;
    vel_marker3.id = 3;
  
    vel_marker2.color.a *= (.5);
    vel_marker3.color.a *= pow((.5),2);

    vel_marker2.pose.position.x += delta_t * vel.pose.position.x;
    vel_marker2.pose.position.y += delta_t * vel.pose.position.y;

    vel_marker3.pose.position.x += 2 * delta_t * vel.pose.position.x;
    vel_marker3.pose.position.y += 2 * delta_t * vel.pose.position.y;

    double vel_mag = sqrt(pow(vel.pose.position.x,2) + pow(vel.pose.position.y,2));
    double mark_scale = delta_t * vel_mag;

    vel_marker2.scale.x += mark_scale;
    vel_marker2.scale.y += mark_scale;

    vel_marker3.scale.x += 2 * mark_scale;
    vel_marker3.scale.y += 2 * mark_scale;

    mark_arr.markers.push_back(vel_marker1);
    mark_arr.markers.push_back(vel_marker2);
    mark_arr.markers.push_back(vel_marker3);
    
    //robot marker
    robo_marker = pos_marker;
    robo_marker.id = 10;
    robo_marker.pose.position.x = 1.0;
    robo_marker.pose.position.y = -1.0;
    robo_marker.pose.position.z = 0.0;
  
    // Set the scale of the robo_marker -- 1x1x1 here means 1m on a side
    robo_marker.scale.x = 0.5;
    robo_marker.scale.y = 0.5;
    robo_marker.scale.z = 0.01;
  
    mark_arr.markers.push_back(robo_marker);

    double v1_dist = sqrt(pow(vel_marker1.pose.position.x-
			      robo_marker.pose.position.x,2) 
			  + pow(vel_marker1.pose.position.y
				-robo_marker.pose.position.y,2));
    double v2_dist = sqrt(pow(vel_marker2.pose.position.x-
			      robo_marker.pose.position.x,2) 
			  + pow(vel_marker2.pose.position.y
				-robo_marker.pose.position.y,2));
    double v3_dist = sqrt(pow(vel_marker3.pose.position.x-
			      robo_marker.pose.position.x,2) 
			  + pow(vel_marker3.pose.position.y
				-robo_marker.pose.position.y,2));
    
    double v1_allowed = ((vel_marker1.scale.x+robo_marker.scale.x) 
			 + (vel_marker1.scale.y+robo_marker.scale.y))/(2*2);//diameter
    double v2_allowed = ((vel_marker2.scale.x+robo_marker.scale.x) 
			 + (vel_marker2.scale.y+robo_marker.scale.y))/(2*2);//diameter
    double v3_allowed = ((vel_marker3.scale.x+robo_marker.scale.x) 
			 + (vel_marker3.scale.y+robo_marker.scale.y))/(2*2);//diameter
    
    double vel_mag2 = sqrt(pow(vel.pose.position.x,2) 
			   + pow(vel.pose.position.y,2));
    double hum_rob_dist = sqrt(pow(pos_marker.pose.position.x-
				   robo_marker.pose.position.x,2) 
			       + pow(pos_marker.pose.position.y
				-robo_marker.pose.position.y,2));
    //printf("dist %f vel %f\n", hum_rob_dist, vel_mag2);
    //if (vel_mag2 > 0.032)
    // if (hum_rob_dist < 1.7)
    //    pubbed_state = front_slow;
    // if (hum_rob_dist < 1.0)
    //    pubbed_state = backward_only;
    // if (vel_mag2 > 0.040)
    //    pubbed_state = backward_only;
  
    if(v3_dist < v3_allowed)
      pubbed_state = front_slow;
    if(v2_dist < v2_allowed)
      pubbed_state = backward_only;
  }    
  
  
  //Color Robot according to the state
  if(mark_arr.markers.size()>0){

    switch(pubbed_state){
    case all_allowed:
      mark_arr.markers[robo_marker_id].color.r = 0.0f;
      mark_arr.markers[robo_marker_id].color.g = 1.0f;
      mark_arr.markers[robo_marker_id].color.b = 0.0f;
      mark_arr.markers[robo_marker_id].color.a = 1.0;
      break;
  
    case front_slow:
      mark_arr.markers[robo_marker_id].color.r = 1.0f;
      mark_arr.markers[robo_marker_id].color.g = 1.0f;
      mark_arr.markers[robo_marker_id].color.b = 0.0f;
      mark_arr.markers[robo_marker_id].color.a = 1.0;
      break;
  
    case backward_only:
      mark_arr.markers[robo_marker_id].color.r = 1.0f;
      mark_arr.markers[robo_marker_id].color.g = 0.0f;
      mark_arr.markers[robo_marker_id].color.b = 0.0f;
      mark_arr.markers[robo_marker_id].color.a = 1.0;
      break;
    }
  }
  
  // Publish the mark_arr.markers[robo_marker_id]
  viz_pub.publish(mark_arr);
  
  return pubbed_state;
}

#define HISTLEN 10
cv::Point2f hist[HISTLEN];
int cur_hist_ind = 0;

void filter_estimate_avg(const cv::Point2f prev_pos, const cv::Point2f prev_vel, 
		     cv::Point2f &cur_pos, cv::Point2f &cur_vel)
{
    hist[cur_hist_ind%HISTLEN] = cur_pos;
    cur_hist_ind++;
    cur_pos = cur_pos*0.;
    if(cur_hist_ind > HISTLEN) {
        for(int i=0;i<HISTLEN;i++)
          cur_pos += hist[i];
        cur_pos.x /= HISTLEN;
        cur_pos.y /= HISTLEN;
    }
    cur_vel = cur_pos - prev_pos;
}

void filter_estimate(const cv::Point2f prev_pos, const cv::Point2f prev_vel, 
		     cv::Point2f &cur_pos, cv::Point2f &cur_vel)
{
  
  if (!isnan(prev_pos.x) && !isnan(prev_vel.x)){
    cur_vel = cur_pos - prev_pos;
    
    cv::Point2f cur_acc = cur_vel-prev_vel;
    double mag_cur_acc = cv::norm(cur_acc);
    
    if (mag_cur_acc>max_filt_acc){
      cur_acc = (max_filt_acc/cv::norm(cur_acc)) * cur_acc;
    }
    cur_vel = prev_vel + cur_acc;
   double mag_cur_vel = cv::norm(cur_vel);
   
   if (mag_cur_vel>max_filt_vel){
     cur_vel = (max_filt_vel/cv::norm(cur_vel)) * cur_vel;
   }
   
   cur_pos = prev_pos + cur_vel;
  }
  
  else{
    if (isnan(prev_pos.x)){
      cur_vel.x = numeric_limits<double>::quiet_NaN();
      cur_vel.y = numeric_limits<double>::quiet_NaN();
      return;
    }
    else{
      cur_vel = cur_pos - prev_pos;
     
      double mag_cur_vel = cv::norm(cur_vel);
     
      if (mag_cur_vel>max_filt_vel){
	cur_vel = (max_filt_vel/cv::norm(cur_vel)) * cur_vel;
      }
      
      cur_pos = prev_pos + 0.5 * cur_vel; // not very sure of velocity
      // the first time around
    }
  }
}
