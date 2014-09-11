#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/concatenate.h>
#include <pcl/common/common.h>
#include <pcl/pcl_config.h>
#include <pcl/visualization/pcl_visualizer.h> 
#include <pcl/conversions.h> 
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include<iostream>


using namespace cv;  
using namespace std; 



typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
boost::mutex cloud_mutex;

ros::Publisher depth_pub;
ros::Publisher rgb_pub;  



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool pc_to_img_no_filter( const PointCloudT::Ptr& cloud, cv::Mat& d_rgb,
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
if (d_depth.size()!=cv::Size(pc_rows, pc_cols) || d_depth.depth()!= CV_32FC1)
d_depth = cv::Mat::zeros(pc_rows, pc_cols, CV_32FC1);
else
d_depth = cv::Scalar(0.0);
if (d_dmask.size()!=cv::Size(pc_rows, pc_cols) || d_dmask.depth()!= CV_8U)
d_dmask = cv::Mat::zeros(pc_rows, pc_cols, CV_8U);
else
d_dmask = cv::Scalar(0);
for (int r=0; r<pc_rows; r++){
cv::Vec3b* rgb_r = d_rgb.ptr<cv::Vec3b> (r);
float* depth_r = d_depth.ptr<float> (r);
unsigned char* dmask_r = d_dmask.ptr<unsigned char> (r);
for (int c=0; c<pc_cols; c++){
PointT point = cloud->at(c,r);
//set depth and depth-mask if not NaN

depth_r[c] = static_cast<float>(point.z);
dmask_r[c] = static_cast<unsigned char>(255);

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


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_in, PointCloudT::Ptr& cloud_out, bool* new_cloud_available_flag){
cloud_mutex.lock();

pcl::PCLPointCloud2 pcl_pc;
pcl_conversions::toPCL(*cloud_in, pcl_pc);
pcl::fromPCLPointCloud2(pcl_pc, *cloud_out);

cv::Mat depth_mat;
cv::Mat rgb_mat;
cv::Mat mask; 
pc_to_img_no_filter(cloud_out, rgb_mat, depth_mat, mask);



cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
depth_mat.copyTo(cv_ptr->image);
depth_pub.publish(cv_ptr->toImageMsg());


cv::imshow("Subtracted Display", cv_ptr->image);



cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
rgb_mat.copyTo(cv_ptr->image);
rgb_pub.publish(cv_ptr->toImageMsg());

*new_cloud_available_flag = true;

cloud_mutex.unlock();
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


int main (int argc, char** argv)
{

ros::init(argc, argv, "listener_recorded");
ros::NodeHandle nh;


cv::namedWindow("Subtracted Display");
cvStartWindowThread();
depth_pub = nh.advertise<sensor_msgs::Image> ("/recorded/depth_image", 1);
rgb_pub = nh.advertise<sensor_msgs::Image>("/recorded/rgb_image", 1); 
/*

ros::Publisher pub = nh.advertise<std_msgs::String>("/rand",1); 
ros::Rate loop_rate(10);
while(ros::ok()){
std_msgs::String abc;
abc.data = "abc";
pub.publish(abc);
loop_rate.sleep();

}*/

bool new_cloud_available_flag = false;

PointCloudT::Ptr cloud (new PointCloudT);

boost::function<void (const sensor_msgs::PointCloud2ConstPtr& )> f = boost::bind (&cloud_cb, _1,  cloud, &new_cloud_available_flag);
ros::Subscriber sub = nh.subscribe("/read_pcd", 1000, f);



ros::spin();



return 0;
}

