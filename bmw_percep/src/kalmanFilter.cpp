#include <bmw_percep/kalmanFilter.hpp>

/**
   Class *implementation* For Kalman Filter with state
   (x,y,vx,vy) and measurement (x,y)
**/

void KalmanFilter::predict(float delta_t){
  x_k1_ = x_k_n_;
  if (delta_t<=0)
    delta_t = delta_t_;
  //if exact delta-t not known use default
  if (delta_t!=delta_t_){
    delta_t_ = delta_t;
    //Recompute A
    A_ = Eigen::Matrix4f::Identity();
    A_(0,2) = delta_t_;
    A_(1,3) = delta_t_;
  }

  //project the state ahead
  x_k_p_ = A_*x_k1_;
  
  //project the error covariance
  P_k1_ = P_k_n_;
  P_k_p_ = A_ * P_k1_ * A_.transpose() + Q_;
  
}

void KalmanFilter::correct(Eigen::Vector2f z_k){
  //Compute Kalman gain
  K_ = P_k_p_ * H_.transpose() * (H_ * P_k_p_ * H_.transpose() 
				  + R_).inverse();
  //Update estimate with observation
  x_k_n_ = x_k_p_ + K_* (z_k - H_ * x_k_p_);

  //Update error cov
  I = Eigen::Matrix4f::Identity();
    P_k_n_ = ( I - (K_ * H_)) * P_k_p_;
}
