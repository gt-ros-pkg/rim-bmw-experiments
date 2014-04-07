#include<iostream>
#include<pcl/io/openni_grabber.h>
#include<opencv2/opencv.hpp>
#include<rim_bmw/cv_utils.hpp>
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


int main(int argc, char** argv)
{

  //Various objects
  PointCloudT::Ptr cloud (new PointCloudT);
  cv::BackgroundSubtractMOG2 cvBg;
  int bg_history = 500;
  //set history
  cvBg.setHistory(bg_history);

  //Tracker params
  int min_human_pixels = 200; // min. pixels for blob to be human

  double init_learn=-1; // learning rate for initialization
  double general_learn=-1; // after initial period
  char win_key=0;

  cv::Mat rgb_im, depth_im, valid_depth, foreMask;
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
  new_cloud_available_flag = false;

  int n_frames=0;
  for (;;){
    n_frames++;
    cloud_mutex.lock ();    // for not overwriting the point cloud
    // chain of events start
   
    //get image, depth-map, valid mask
    cv_utils::pc_to_img(cloud, rgb_im, depth_im, valid_depth);

    cout << "\nInitializing background ..." << endl;
    cout << "\nPress q to exit initialization" << endl;

    //back-ground initialize
    if (!bg_init){
      cv::imshow("Init", rgb_im);
      cvBg.apply(rgb_im, foreMask, init_learn);
      win_key = cv::waitKey(10);

      //if more than history frames seen for initialization
      if (bg_history < n_frames){
	//set new history limit
	bg_history = n_frames;
	cvBg.setHistory(bg_history);
      }

      //Quit or Not
      if (win_key!='q'){
	// no further processing required
	continue;
      }
      else{
	bg_init = true;
	//to the next frame we go..
	continue;
      }
    }
    
    //get foreground mask without learning from image
    cvBg.apply(rgb_im, foreMask, 0);
    
    //morphological operations

    //find human component in foreground

    //add in non-human as background

    cloud_mutex.unlock (); // let go of current cloud
  }

  return 0; // successfully exit
}
