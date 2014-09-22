
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

#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Eigen>
#include <Eigen/Sparse>
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

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/range/algorithm/random_shuffle.hpp>

#define EMF_DEBUG

using namespace std;

#define NUM_KIN 2

typedef pcl::PointXYZ PXYZ;
typedef pcl::PointCloud<PXYZ> PCXYZ;

typedef pcl::PointXYZRGB PRGB;
typedef pcl::PointCloud<PRGB> PCRGB;

typedef Eigen::Array<bool, 1, Eigen::Dynamic> VectorXBool;
typedef Eigen::Triplet<uint8_t> TripUInt8;
typedef Eigen::SparseMatrix<uint8_t> SMatUInt8;
typedef std::vector<Eigen::Affine2f> AffineList;

inline bool 
getEllipseTangentPoints(float a, float b, const Eigen::Vector2f& cam_pt, 
                        Eigen::Vector2f& t1, Eigen::Vector2f& t2);

inline VectorXBool
checkPointsBetweenVectors(const Eigen::Vector2f& t1, const Eigen::Vector2f& t2, 
                          const Eigen::MatrixXf& pts);

inline void 
stocasticUniversalSampling(boost::mt19937& randng, Eigen::ArrayXf& weights, 
                           std::vector<int>& samples);

class ExpMaxPersonFilter
{
public:
  ExpMaxPersonFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

protected:
  void recvPCCallback(const PCRGB::ConstPtr& msg);
  void pubImages();
  inline void circleCameras(cv::Mat& img, cv::Scalar scalar);
  void convertTripletsToDense(std::vector<TripUInt8>& trip_list, Eigen::VectorXf& pc_proj_hist,
                              Eigen::MatrixXf& proj_pts);
  inline void drawParticles(cv::Mat& img, cv::Scalar scalar, AffineList& affs);
  float evalEllipse(Eigen::Affine2f& ell_affine, Eigen::VectorXf& pts_hist, 
                    Eigen::MatrixXf& pts, Eigen::Vector2f& kin_pt);
  void sampleMotionModel(AffineList& affs_in, AffineList& affs_out);

  ros::Subscriber pc_sub_;
  ros::Publisher pc1_pub_;
  ros::Publisher pc2_pub_;
  image_transport::ImageTransport it_;
  std::vector<image_transport::CameraPublisher> cam_pubs_;
  std::vector<cv_bridge::CvImagePtr> img_cv_ptrs_;
  sensor_msgs::CameraInfoPtr cam_info_;
  image_geometry::PinholeCameraModel cam_model_;
  std_msgs::Header cam_header_;

  tf::StampedTransform cam_stamped_tf_;
  tf::Vector3 cam_offset_;
  tf::TransformBroadcaster tf_bcast_;
  tf::TransformListener tf_list_;
  tf::StampedTransform kin1_tf_, kin2_tf_;
  tf::Vector3 kin1_pt_, kin2_pt_; // in the base link's frame
  tf::Vector3 kin1_pt_cam_, kin2_pt_cam_; // in the simulated camera's frame

  cv::Mat img_proj_;
  cv::Mat img_conv_;
  cv::Mat img_sensor_mdl_;

  std::vector<TripUInt8> trip_lists_[NUM_KIN];

  double ell_width_, ell_depth_, ell_surf_sigma_;

  boost::mt19937 randng_;
  AffineList particle_affs_;
  Eigen::ArrayXf particle_weights_;
  double momodel_x_mean_, momodel_x_sigma_;
  double momodel_y_mean_, momodel_y_sigma_;
  double momodel_r_mean_, momodel_r_sigma_;
};

ExpMaxPersonFilter::
ExpMaxPersonFilter(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
  : it_(ros::NodeHandle(nh_priv, "overhead_cams")), cam_info_(new sensor_msgs::CameraInfo())
{
  cam_header_.seq = 0;
  cam_header_.frame_id = "/sim_overhead_cam_frame";

  int img_height, img_width;
  double img_span_width, img_span_height;
  double sim_cam_x = 0.0, sim_cam_y = 0.0, sim_cam_z = 0.0;
  nh_priv.getParam("sim_image_width", img_width);
  nh_priv.getParam("sim_image_height", img_height);
  nh_priv.getParam("sim_image_span_width", img_span_width);
  nh_priv.getParam("sim_image_span_height", img_span_height);
  nh_priv.getParam("sim_cam_x", sim_cam_x); 
  nh_priv.getParam("sim_cam_y", sim_cam_y); 
  nh_priv.getParam("sim_cam_z", sim_cam_z); 

  nh_priv.getParam("ell_width", ell_width_); 
  nh_priv.getParam("ell_depth", ell_depth_); 
  nh_priv.getParam("ell_surf_sigma", ell_surf_sigma_); 

  // initialize simulated camera info (intrinsics)
  cam_info_->height = img_height;
  cam_info_->width = img_width;
  {
    float cx = img_width/2.0, cy = img_height/2.0;
    float fx = img_width*sim_cam_z/img_span_width;
    float fy = img_height*sim_cam_z/img_span_height;
    boost::array<float,9> K = { fx, 0.0,  cx, 
                               0.0,  fy,  cy, 
                               0.0, 0.0, 1.0};
    boost::array<float,9> R = {1.0, 0.0, 0.0, 
                               0.0, 1.0, 0.0, 
                               0.0, 0.0, 1.0};
    boost::array<float,12> P = { fx, 0.0,  cx, 0.0,
                                0.0,  fy,  cy, 0.0,
                                0.0, 0.0, 1.0, 0.0};
    cam_info_->K = K;
    cam_info_->R = R;
    cam_info_->distortion_model = "plumb_bob";
    cam_info_->D.resize(5, 0.0);
    cam_info_->P = P;
  }
  cam_model_.fromCameraInfo(cam_info_);
  ///////////////////////////////////////////////////
  
  // initialize simulated camera transform (extrinsics)
  std::string base_link;
  nh_priv.getParam("base_link", base_link);
  tf::Quaternion cam_rot; cam_rot.setEuler(0, M_PI, 0);
  tf::Transform cam_tf(cam_rot, tf::Vector3(sim_cam_x, sim_cam_y, sim_cam_z));
  cam_stamped_tf_ = tf::StampedTransform(cam_tf, ros::Time(), base_link, cam_header_.frame_id);
  cam_offset_ = cam_stamped_tf_.getOrigin();
  ///////////////////////////////////////////////////

  // initialize simulated images
  img_proj_ = cv::Mat(img_width, img_height, CV_8UC1, cv::Scalar(0));
  img_conv_ = cv::Mat(img_width, img_height, CV_16SC1, cv::Scalar(0));
  img_sensor_mdl_ = cv::Mat(img_width, img_height, CV_32FC1, cv::Scalar(0.0f));
  int num_cams = 3;
  img_cv_ptrs_.resize(num_cams);
  img_cv_ptrs_[0].reset(new cv_bridge::CvImage(cam_header_, "mono8", img_proj_));
  img_cv_ptrs_[1].reset(new cv_bridge::CvImage(cam_header_, "16SC1", img_conv_));
  img_cv_ptrs_[2].reset(new cv_bridge::CvImage(cam_header_, "32FC1", img_sensor_mdl_));
  ///////////////////////////////////////////////////

  // get kinect sensor transforms
  std::string kin1_frame, kin2_frame;
  nh_priv.getParam("kinect_1_frame", kin1_frame);
  nh_priv.getParam("kinect_2_frame", kin2_frame);
  tf_list_.waitForTransform(kin1_frame, base_link, ros::Time(0), ros::Duration(2.0));
  tf_list_.waitForTransform(kin2_frame, base_link, ros::Time(0), ros::Duration(2.0));
  tf_list_.lookupTransform(base_link, kin1_frame, ros::Time(0), kin1_tf_);
  tf_list_.lookupTransform(base_link, kin2_frame, ros::Time(0), kin2_tf_);
  kin1_pt_ = kin1_tf_.getOrigin();
  kin2_pt_ = kin2_tf_.getOrigin();
  kin1_pt_cam_ =
    tf::Vector3(kin1_pt_.x()-cam_offset_.x(), -kin1_pt_.y()+cam_offset_.y(), cam_offset_.z());
  kin2_pt_cam_ =
    tf::Vector3(kin2_pt_.x()-cam_offset_.x(), -kin2_pt_.y()+cam_offset_.y(), cam_offset_.z());
  ///////////////////////////////////////////////////

  trip_lists_[0].reserve(20000);
  trip_lists_[1].reserve(20000);

  // Particle initialization
  int num_particles;
  nh_priv.getParam("num_particles", num_particles);
  boost::uniform_real<> width_range(-img_span_width/2, img_span_width/2);
  boost::uniform_real<> height_range(-img_span_height/2, img_span_height/2);
  boost::uniform_real<> rot_range(0, 2*M_PI);
  particle_weights_.resize(num_particles);
  for(int i = 0; i < num_particles; i++) {
    particle_affs_.push_back(
        Eigen::Translation2f(width_range(randng_), height_range(randng_)) * 
        Eigen::Rotation2D<float>(rot_range(randng_)));
    particle_weights_(i) = 1.0 / ((float) num_particles);
  }
  nh_priv.getParam("momodel_x_mean", momodel_x_mean_);
  nh_priv.getParam("momodel_x_sigma", momodel_x_sigma_);
  nh_priv.getParam("momodel_y_mean", momodel_y_mean_);
  nh_priv.getParam("momodel_y_sigma", momodel_y_sigma_);
  nh_priv.getParam("momodel_r_mean", momodel_r_mean_);
  nh_priv.getParam("momodel_r_sigma", momodel_r_sigma_);
  ///////////////////////////////////////////////////

  // initialize ros topics
  pc1_pub_ = nh_priv.advertise<PCRGB>("pc_out_1", 1);
  pc2_pub_ = nh_priv.advertise<PCRGB>("pc_out_2", 1);
  for(int i = 0; i < num_cams; i++)
    cam_pubs_.push_back(it_.advertiseCamera("cam_pub_" + boost::lexical_cast<std::string>(i+1), 1));
  pc_sub_ = nh.subscribe<PCRGB>("pc_in", 1, &ExpMaxPersonFilter::recvPCCallback, this);
  ///////////////////////////////////////////////////
}

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

inline void ExpMaxPersonFilter::circleCameras(cv::Mat& img, cv::Scalar scalar)
{
  tf::Matrix3x3 kin1_rot = kin1_tf_.getBasis();
  tf::Matrix3x3 kin2_rot = kin2_tf_.getBasis();
  tf::Vector3 kin1_z_axis = kin1_rot.getColumn(2);
  tf::Vector3 kin2_z_axis = kin2_rot.getColumn(2);
  tf::Vector3 kin1_z_axis_normed(kin1_z_axis.x(), -kin1_z_axis.y(), 0.0);
  tf::Vector3 kin2_z_axis_normed(kin2_z_axis.x(), -kin2_z_axis.y(), 0.0);
  kin1_z_axis_normed.normalize();
  kin2_z_axis_normed.normalize();

  cv::Point2d kin1_pt2d = cam_model_.project3dToPixel(
    cv::Point3d(kin1_pt_cam_.x(), kin1_pt_cam_.y(), kin1_pt_cam_.z()));
  cv::Point2d kin2_pt2d = cam_model_.project3dToPixel(
    cv::Point3d(kin2_pt_cam_.x(), kin2_pt_cam_.y(), kin2_pt_cam_.z()));
  cv::circle(img, kin1_pt2d, 6, scalar, -1);
  cv::circle(img, kin2_pt2d, 6, scalar, -1);
  cv::line(img, kin1_pt2d, kin1_pt2d+20*cv::Point2d(kin1_z_axis_normed.x(), kin1_z_axis_normed.y()), 
           scalar, 3);
  cv::line(img, kin2_pt2d, kin2_pt2d+20*cv::Point2d(kin2_z_axis_normed.x(), kin2_z_axis_normed.y()), 
           scalar, 3);
}

inline void ExpMaxPersonFilter::drawParticles(cv::Mat& img, cv::Scalar scalar, AffineList& affs)
{
  AffineList::iterator aff_it;
  for(aff_it = affs.begin(); aff_it != affs.end(); ++aff_it) {
    cv::Point2d pt2d = cam_model_.project3dToPixel(
      cv::Point3d(aff_it->translation()(0), aff_it->translation()(1), cam_offset_.z()));
    cv::circle(img, pt2d, 6, scalar, -1);
    Eigen::Matrix2f rot_mat = aff_it->rotation();
    cv::line(img, pt2d, pt2d+20*cv::Point2d(rot_mat(0,1), rot_mat(1,1)), 
             scalar, 3);
  }
}

inline bool 
getEllipseTangentPoints(float a, float b, const Eigen::Vector2f& cam_pt, 
                        Eigen::Vector2f& t1, Eigen::Vector2f& t2)
{
  float px = cam_pt(0), py = cam_pt(1);
  float a2 = a*a, b2 = b*b, px2 = px*px, py2 = py*py;
  if(px2/a2 + py2/b2 <= 1)
    return false; // camera point inside ellipse; no tangents

  if(std::fabs(py) > 0.001) {
    float Ax = (py2 + px2*(b2/a2));
    float Bx = (-2*px*b2);
    float Cx = (a2*b2-py2*a2);
    float Bx2 = Bx*Bx;

    t1(0) = (-Bx + sqrt(Bx2 - 4*Ax*Cx)) / (2*Ax);
    t2(0) = (-Bx - sqrt(Bx2 - 4*Ax*Cx)) / (2*Ax);
    t1(1) = (-px/py*b2/a2) * t1(0) + b2/py;
    t2(1) = (-px/py*b2/a2) * t2(0) + b2/py;
  }
  else {
    float Ay = (px2 + py2*(a2/b2));
    float By = (-2*py*a2);
    float Cy = (b2*a2-px2*b2);
    float By2 = By*By;

    t1(1) = (-By + sqrt(By2 - 4*Ay*Cy)) / (2*Ay);
    t2(1) = (-By - sqrt(By2 - 4*Ay*Cy)) / (2*Ay);
    t1(0) = (-py/px*a2/b2) * t1(1) + a2/px;
    t2(0) = (-py/px*a2/b2) * t2(1) + a2/px;
  }
  return true;
}

inline VectorXBool
checkPointsBetweenVectors(const Eigen::Vector2f& t1, const Eigen::Vector2f& t2, 
                          const Eigen::MatrixXf& pts)
{
  Eigen::Vector2f t1cross(-t1(1), t1(0));
  Eigen::Vector2f t2cross(t2(1), -t2(0));
  if(t2.dot(t1cross) > 0) {
    // the cross product t1 x t2 > 0; the smaller angle from t1 to t2 is positive

    // these find the sign of the angles t1 to pts and pts to t2 respectively
    // if both of these angles are positive then the point is between the two vectors
    //                 t1 x pt > 0                                 pt x t2 > 0
    return ((t1cross.transpose()*pts).array() > 0) && ((t2cross.transpose()*pts).array() > 0);
  }
  else {
    // the cross product t1 x t2 < 0; the smaller angle from t1 to t2 is negative

    // these find the sign of the angles t1 to pts and pts to t2 respectively
    // if both of these angles are negative then the point is between the two vectors
    //                 t1 x pt < 0                                 pt x t2 < 0
    return ((t1cross.transpose()*pts).array() < 0) && ((t2cross.transpose()*pts).array() < 0);
  }
}

void ExpMaxPersonFilter::
convertTripletsToDense(std::vector<TripUInt8>& trip_list, Eigen::VectorXf& pc_proj_hist,
                       Eigen::MatrixXf& proj_pts)
{
  SMatUInt8 proj_mat(cam_info_->height, cam_info_->width);
  proj_mat.setFromTriplets(trip_list.begin(), trip_list.end());
  trip_list.clear();

  // convert histogram to dense form: vector of values and matrix of locations
  pc_proj_hist.resize(proj_mat.nonZeros());
  proj_pts.resize(2, proj_mat.nonZeros());
  int hist_ind = 0;
  for (int i = 0; i < proj_mat.outerSize(); ++i) {
    for (SMatUInt8::InnerIterator mat_it(proj_mat, i); mat_it; ++mat_it) {
      // stores the count of points at each location
      pc_proj_hist(hist_ind) = mat_it.value();

      // these points are in the simulated camera frame and on the floor
      cv::Point2d pt2d(mat_it.col(), mat_it.row());
      cv::Point3d pt3d = cam_model_.projectPixelTo3dRay(pt2d)*cam_offset_.z();
      proj_pts(0, hist_ind) = pt3d.x; 
      proj_pts(1, hist_ind) = pt3d.y; 
      hist_ind++;
    }
  }
}

inline float ExpMaxPersonFilter::
evalEllipse(Eigen::Affine2f& ell_affine, Eigen::VectorXf& pts_hist, 
            Eigen::MatrixXf& pts, Eigen::Vector2f& kin_pt)
{
  float ell_a = ell_width_/2, ell_b = ell_depth_/2;

  Eigen::Vector2f kin_tang1, kin_tang2;
  if(!getEllipseTangentPoints(ell_a, ell_b, kin_pt, kin_tang1, kin_tang2)) 
    return 0.0;

  Eigen::MatrixXf pts_trans = ell_affine*pts.colwise().homogeneous();
  Eigen::MatrixXf pts_trans_scaled = Eigen::AlignedScaling2f(1/ell_a, 1/ell_b)*pts_trans;
  // something on this \/ line is really slow. probably the array transform
  Eigen::ArrayXf dists = (pts_trans_scaled.colwise().squaredNorm().array() - 1).square();
  Eigen::ArrayXf weights = (dists/(-ell_surf_sigma_*ell_surf_sigma_)).exp();

  VectorXBool pts_visible = checkPointsBetweenVectors(kin_tang1, kin_tang2, pts_trans);
  weights = weights*pts_visible.cast<float>().transpose();
  // float max_weight = weights.maxCoeff();
  return weights.matrix().dot(pts_hist);
}

inline void stocasticUniversalSampling(boost::mt19937& randng, Eigen::ArrayXf& weights, 
                                       std::vector<int>& samples)
{
  // weights should sum to 1
  int nsamps = weights.size();
  float ninv = 1.0 / ((float) nsamps);
  Eigen::ArrayXf cdf(nsamps);

  // cumsum
  cdf(0) = weights(0);
  for(int i = 1; i < nsamps; ++i) 
    cdf(i) = cdf(i-1) + weights(i); 
  boost::uniform_real<> thresh_range(0.0, ninv);
  float thresh = thresh_range(randng);

  int cdf_ind = 0;
  samples.resize(nsamps);
  for(int i = 0; i < nsamps; ++i) {
    // generate samples
    while(thresh > cdf(cdf_ind))
      cdf_ind++;
    samples[i] = cdf_ind;
    weights(i) = ninv;
    thresh += ninv;
  }
}

void ExpMaxPersonFilter::
sampleMotionModel(AffineList& affs_in, AffineList& affs_out)
{
  boost::normal_distribution<> x_distrib(momodel_x_mean_, momodel_x_sigma_);
  boost::variate_generator<boost::mt19937&, 
                           boost::normal_distribution<> > x_gen(randng_, x_distrib);
  boost::normal_distribution<> y_distrib(momodel_y_mean_, momodel_y_sigma_);
  boost::variate_generator<boost::mt19937&, 
                           boost::normal_distribution<> > y_gen(randng_, y_distrib);
  boost::normal_distribution<> r_distrib(momodel_r_mean_, momodel_r_sigma_);
  boost::variate_generator<boost::mt19937&, 
                           boost::normal_distribution<> > r_gen(randng_, r_distrib);
  for(int i = 0; i < affs_in.size(); i++) {
    affs_out.push_back(
        affs_in[i] *
        Eigen::Translation2f(x_gen(), y_gen()) * 
        Eigen::Rotation2D<float>(r_gen())
        );
  }
}

void ExpMaxPersonFilter::recvPCCallback(const PCRGB::ConstPtr& msg)
{
  static ros::Duration sum_time;
  static int num_times = 0;
  ros::Time start_time = ros::Time::now();
  cv::Rect img_rect(cv::Point(), img_proj_.size());
  // img_proj_.setTo(cv::Scalar(0));

  // convert to histogram on ground plane
  // uint32_t color_red = 0x00ff0000;
  // float color_red_float = *reinterpret_cast<float*>(&color_red); 
  // PCRGB::const_iterator pt_it;
  // for(pt_it = msg->begin(); pt_it != msg->end(); ++pt_it) {
  //   // these points are projected into the simulated camera's frame
  //   cv::Point2d pt2d = cam_model_.project3dToPixel(
  //           cv::Point3d(pt_it->x-cam_offset_.x(), cam_offset_.y()-pt_it->y, cam_offset_.z()));
  //   if(img_rect.contains(pt2d))
  //     trip_lists_[pt_it->rgb == color_red_float].push_back(TripUInt8(pt2d.y, pt2d.x, 1));
  //     // img_proj_.at<uint8_t>(pt2d.y, pt2d.x) += 1; 
  // }
  // ROS_INFO_THROTTLE(1.0, "trip_lists_ sizes: %d %d", trip_lists_[0].size(), trip_lists_[1].size());

  // Eigen::VectorXf pc_proj_hists[NUM_KIN];
  // Eigen::MatrixXf proj_pts[NUM_KIN];
  // for(int kin_ind = 0; kin_ind < NUM_KIN; ++kin_ind)
  //   convertTripletsToDense(trip_lists_[kin_ind], pc_proj_hists[kin_ind], proj_pts[kin_ind]);
  //////////////////////////////////////////////////

  // Eigen::Vector2f kin_pts[NUM_KIN];
  // kin_pts[0] = Eigen::Vector2f(kin1_pt_cam_.x(), kin1_pt_cam_.y());
  // kin_pts[1] = Eigen::Vector2f(kin2_pt_cam_.x(), kin2_pt_cam_.y());

  AffineList affs_moved;
  sampleMotionModel(particle_affs_, affs_moved);
  particle_affs_ = affs_moved;
  img_sensor_mdl_.setTo(cv::Scalar(0.0));
  drawParticles(img_sensor_mdl_, cv::Scalar(1.0), particle_affs_);
  pubImages();

  // Eigen::Affine2f ell_affine = Eigen::Rotation2D<float>(0.3) * Eigen::Translation2f(0.0, 1.0);
  // for(int i = 0; i < 100; ++i) 
  //   for(int kin_ind = 0; kin_ind < NUM_KIN; ++kin_ind)
  //     evalEllipse(ell_affine, pc_proj_hists[kin_ind], proj_pts[kin_ind], kin_pts[kin_ind]);
  sum_time += ros::Time::now() - start_time;
  ROS_INFO_THROTTLE(1.0, "avg time: %f", sum_time.toSec()/++num_times);
#if 0
  Eigen::MatrixXf all_proj_pts(2, 1000*1000);
  for(int i = 0; i < 1000; ++i) {
    for(int j = 0; j < 1000; ++j) {
      cv::Point2d pt2d(j, i);
      cv::Point3d pt3d = cam_model_.projectPixelTo3dRay(pt2d)*cam_offset_.z();
      all_proj_pts(0, j+i*1000) = pt3d.x; 
      all_proj_pts(1, j+i*1000) = pt3d.y; 
    }
  }

  float ell_a = ell_width_/2, ell_b = ell_depth_/2;
  Eigen::Affine2f aff = Eigen::Rotation2D<float>(0.3) * Eigen::Translation2f(0.0, 1.0);
  Eigen::MatrixXf all_proj_pts_trans = aff*all_proj_pts.colwise().homogeneous();
  Eigen::MatrixXf all_proj_pts_trans_scaled = 
    Eigen::AlignedScaling2f(1/ell_a, 1/ell_b)*all_proj_pts_trans;
  Eigen::ArrayXf dists = (all_proj_pts_trans_scaled.colwise().squaredNorm().array() - 1).square();
  Eigen::ArrayXf weights = (dists/(-0.1*0.1)).exp();
  float max_weight = weights.maxCoeff();

  Eigen::Vector2f kin1_pt(kin1_pt_cam_.x(), kin1_pt_cam_.y());
  Eigen::Vector2f kin2_pt(kin2_pt_cam_.x(), kin2_pt_cam_.y());
  Eigen::Vector2f kin1_tang1, kin1_tang2, kin2_tang1, kin2_tang2;
  if(getEllipseTangentPoints(ell_a, ell_b, kin1_pt, kin1_tang1, kin1_tang2)) {
    cv::Point2d pt2d1, pt2d2;
    pt2d1 = cam_model_.project3dToPixel(
        cv::Point3d(kin1_tang1(0), kin1_tang1(1), cam_offset_.z()));
    pt2d2 = cam_model_.project3dToPixel(
        cv::Point3d(kin1_tang2(0), kin1_tang2(1), cam_offset_.z()));
    cv::circle(img_sensor_mdl_, pt2d1, 6, cv::Scalar(max_weight), -1);
    cv::circle(img_sensor_mdl_, pt2d2, 6, cv::Scalar(max_weight), -1);
  }
  if(getEllipseTangentPoints(ell_a, ell_b, kin2_pt, kin2_tang1, kin2_tang2)) {
    cv::Point2d pt2d1, pt2d2;
    pt2d1 = cam_model_.project3dToPixel(
        cv::Point3d(kin2_tang1(0), kin2_tang1(1), cam_offset_.z()));
    pt2d2 = cam_model_.project3dToPixel(
        cv::Point3d(kin2_tang2(0), kin2_tang2(1), cam_offset_.z()));
    cv::circle(img_sensor_mdl_, pt2d1, 6, cv::Scalar(max_weight), -1);
    cv::circle(img_sensor_mdl_, pt2d2, 6, cv::Scalar(max_weight), -1);
  }
  circleCameras(img_sensor_mdl_, cv::Scalar(max_weight));

  VectorXBool pts_visible1 = checkPointsBetweenVectors(kin2_tang1, kin2_tang2, all_proj_pts_trans);
  // weights = weights*pts_visible1.cast<float>().transpose();

  int weight_ind = 0;
  for(int j = 0; j < 1000; ++j)
    for(int i = 0; i < 1000; ++i)
      img_sensor_mdl_.at<float>(j, i) = weights(weight_ind++);
  
  // AffineList::iterator aff_it;
  // for(aff_it = particle_affs_.begin(); aff_it != particle_affs_.end(); ++aff_it) {
  //   (*aff_it)*proj_pts.colwise().homogeneous();
  // }
  

  // ROS_INFO_THROTTLE(1.0, "PC size: %d", msg->size());
  // ROS_INFO_THROTTLE(1.0, "NNZ: %d", cv::countNonZero(img_proj_));
  // printMinMax(img_proj_);
  // circleCameras(img_proj_);
  pubImages();
#endif

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
