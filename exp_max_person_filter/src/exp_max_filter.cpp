#include <stdio.h>
#include <iostream>
#include <algorithm>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define EMF_DEBUG

using namespace std;

typedef pcl::PointXYZ PXYZ;
typedef pcl::PointCloud<PXYZ> PCXYZ;

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

#define IMG_HEIGHT 1000
#define IMG_WIDTH 1000

class ExpMaxPersonFilter
{
public:
  ExpMaxPersonFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
    : base_link_("/table_link"),
      it_(ros::NodeHandle(nh_priv, "overhead_cams")), cam_info_(new sensor_msgs::CameraInfo()),
      img_proj_(IMG_WIDTH, IMG_HEIGHT, CV_8UC1, cv::Scalar(0)),
      img_conv_(IMG_WIDTH, IMG_HEIGHT, CV_16SC1, cv::Scalar(0))
  {
    cam_header_.seq = 0;
    cam_header_.frame_id = "/sim_overhead_cam_frame";

    cam_info_->height = IMG_HEIGHT;
    cam_info_->width = IMG_WIDTH;
    {
      float f = 18000.0, cx = IMG_WIDTH/2.0, cy = IMG_HEIGHT/2.0;
      boost::array<float,9> K = {  f, 0.0,  cx, 
                                 0.0,   f,  cy, 
                                 0.0, 0.0, 1.0};
      boost::array<float,9> R = {1.0, 0.0, 0.0, 
                                 0.0, 1.0, 0.0, 
                                 0.0, 0.0, 1.0};
      boost::array<float,12> P = {  f, 0.0,  cx, 0.0,
                                  0.0,   f,  cy, 0.0,
                                  0.0, 0.0, 1.0, 0.0};
      cam_info_->K = K;
      cam_info_->R = R;
      cam_info_->distortion_model = "plumb_bob";
      cam_info_->D.resize(5, 0.0);
      cam_info_->P = P;
    }
    cam_model_.fromCameraInfo(cam_info_);
    
    tf::Quaternion cam_rot;
    cam_rot.setRPY(0.0, 0.0, M_PI);
    tf::Transform cam_tf(cam_rot, tf::Vector3(2.0, 2.5, 100.0));
    cam_stamped_tf_ = tf::StampedTransform(cam_tf, ros::Time(), base_link_, cam_header_.frame_id);

    int num_cams = 2;
    img_cv_ptrs_.resize(num_cams);
    img_cv_ptrs_[0].reset(new cv_bridge::CvImage(cam_header_, "mono8", img_proj_));
    img_cv_ptrs_[1].reset(new cv_bridge::CvImage(cam_header_, "16SC1", img_conv_));

    pc1_pub_ = nh_priv.advertise<PCRGB>("pc_out_1", 1);
    pc2_pub_ = nh_priv.advertise<PCRGB>("pc_out_2", 1);
    for(int i = 0; i < num_cams; i++)
      cam_pubs_.push_back(it_.advertiseCamera("cam_pub_" + boost::lexical_cast<std::string>(i+1), 1));
    pc_sub_ = nh.subscribe<PCRGB>("pc_in", 1, &ExpMaxPersonFilter::recvPCCallback, this);
  }
protected:
  void recvPCCallback(const PCRGB::ConstPtr& msg);
  void pubImages();

  ros::Subscriber pc_sub_;
  ros::Publisher pc1_pub_;
  ros::Publisher pc2_pub_;
  image_transport::ImageTransport it_;
  std::vector<image_transport::CameraPublisher> cam_pubs_;
  std::vector<cv_bridge::CvImagePtr> img_cv_ptrs_;
  sensor_msgs::CameraInfoPtr cam_info_;
  image_geometry::PinholeCameraModel cam_model_;
  std_msgs::Header cam_header_;

  std::string base_link_;
  tf::StampedTransform cam_stamped_tf_;
  tf::TransformBroadcaster tf_bcast_;
  tf::TransformListener tf_list_;

  cv::Mat img_proj_;
  cv::Mat img_conv_;
};

void ExpMaxPersonFilter::pubImages()
{
  cam_header_.stamp = ros::Time::now();
  cam_info_->header = cam_header_;

  for(int i = 0; i < img_cv_ptrs_.size(); i++) {
    sensor_msgs::ImagePtr img_msg = img_cv_ptrs_[i]->toImageMsg();
    img_msg->header = cam_header_;
    cam_pubs_[i].publish(img_msg, cam_info_);
  }
  cam_stamped_tf_.stamp_ = cam_header_.stamp;
  tf_bcast_.sendTransform(cam_stamped_tf_);
  cam_header_.seq++;
}

inline uint8_t clampedAdd(uint8_t n)
{
  return n == 255 ? 255 : n+1;
}

inline void printMinMax(cv::Mat& img)
{
  double min_val, max_val;
  cv::Point min_loc, max_loc;
  cv::minMaxLoc(img, &min_val, &max_val, &min_loc, &max_loc);
  ROS_INFO_THROTTLE(1.0, "Min (%d, %d): %d; Max (%d, %d): %d", 
                    min_loc.x, min_loc.y, (int) min_val, max_loc.x, max_loc.y, (int) max_val);
}

void ExpMaxPersonFilter::recvPCCallback(const PCRGB::ConstPtr& msg)
{
  static ros::Duration sum_time;
  static int num_times = 0;
  ros::Time start_time = ros::Time::now();
  cv::Rect img_rect(cv::Point(), img_proj_.size());
  img_proj_.setTo(cv::Scalar(0));
  float cam_offset_x = cam_stamped_tf_.getOrigin().x();
  float cam_offset_y = cam_stamped_tf_.getOrigin().y();
  float cam_offset_z = cam_stamped_tf_.getOrigin().z();
  for(int i = 0; i < msg->size(); i++) {
    PRGB pt = msg->at(i);
    cv::Point2d pt2d = cam_model_.project3dToPixel(
                                cv::Point3d(pt.x-cam_offset_x, -pt.y+cam_offset_y, cam_offset_z));
    if(img_rect.contains(pt2d))
      img_proj_.at<uint8_t>(pt2d.y, pt2d.x) += 1; // clampedAdd(img_proj_.at<uint8_t>(pt2d.y, pt2d.x));
  }
  sum_time += ros::Time::now() - start_time;
  ROS_INFO_THROTTLE(1.0, "avg time: %f", sum_time.toSec()/++num_times);
  ROS_INFO_THROTTLE(1.0, "PC size: %d", msg->size());
  ROS_INFO_THROTTLE(1.0, "NNZ: %d", cv::countNonZero(img_proj_));
  printMinMax(img_proj_);

#if 0
  static cv::Mat conv_kernel;
  if(conv_kernel.total() == 0) {
    int circ_outer_diam = 90;
    int circ_inner_diam = 70;
    cv::Mat outer_circle = 
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(circ_outer_diam, circ_outer_diam));
    cv::Mat inner_circle = 
      cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(circ_inner_diam, circ_inner_diam));
    conv_kernel = outer_circle;
    int diff = (circ_outer_diam - circ_inner_diam) / 2;
    cv::Range rng(diff, circ_outer_diam-diff);
    cv::bitwise_xor(outer_circle(rng, rng), inner_circle, conv_kernel(rng, rng));
  }
  cv::filter2D(img_proj_, img_conv_, CV_16S, conv_kernel);
  pubImages();
#endif
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "exp_max_person_filter");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    
    ExpMaxPersonFilter filter(nh, nh_priv);
    ros::spin();
    return 0;
}
