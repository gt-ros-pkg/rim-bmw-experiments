#include<bmw_percep/groundPlane.hpp>
#include<iostream>
#include<pcl/io/openni_grabber.h>
/**
   Sample program testing the background subtraction after connecting to 
   single RGBD camera.
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
  cloud_mutex.lock ();    // for not overwriting the point cloud from another thread
  *cloud = *callback_cloud;
  *new_cloud_available_flag = true;
  cloud_mutex.unlock ();
}

int main(int argc, char** argv)
{

  // Read Kinect live stream:
  PointCloudT::Ptr cloud (new PointCloudT);
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

  cloud_mutex.lock ();    // for not overwriting the point cloud

  GroundPlane ground_obj(cloud);
  
  //write these to file?
  char yn;
  cout << endl << "Write coefficients to file:(y/n)<return>"; 
  cin >> yn; cout << endl;
  if (yn=='y'){
    //choose file name
    string path = "data/";
    string fileName = "ground_coeffs.txt";
    cout << endl << "file name : " << fileName << " correct? (y/n)<return>"; 
    cin>>yn; cout << endl;
    if(yn!='y'){
      cout << "New name: "; cin >> fileName; cout << endl;
    }
    string total_file = path + fileName;
    ground_obj.writeFile(total_file);
  }

  cloud_mutex.unlock();

  cout << "Locked no more!" << endl;

  return 0;
}
