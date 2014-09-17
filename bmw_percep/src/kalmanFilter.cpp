#include <bmw_percep/kalmanFilter.hpp>

/**
   Class *implementation* For Kalman Filter with state
   (x,y,vx,vy) and measurement (x,y)
**/

void KalmanFilter::predict(float delta_t){
  
  // x_k1_ = x_k_n_;
  
  if (delta_t<=0)
    delta_t = delta_t_;
  //if exact delta-t not known use default
  if (delta_t!=delta_t_){
    delta_t_ = delta_t;
    delta_change();
  }

  //project the state ahead
  x_k_p_ = A_*x_k1_;
  
  //project the error covariance
  // P_k1_ = P_k_n_;
  P_k_p_ = A_ * P_k1_ * A_.transpose() + Q_;
  
}

//Check if things are way off and reinitialize?
void KalmanFilter::correct(Eigen::Vector2f z_k){
  //Compute Kalman gain
  K_ = P_k_p_ * H_.transpose() * (H_ * P_k_p_ * H_.transpose() 
				  + R_).inverse();
  //Update estimate with observation
  x_k_n_ = x_k_p_ + K_* (z_k - H_ * x_k_p_);

  //Update error cov
  Eigen::Matrix4f I = Eigen::Matrix4f::Identity();
    P_k_n_ = ( I - (K_ * H_)) * P_k_p_;
}

//Dummy constructor
KalmanFilter::KalmanFilter(){}

//Real constructor, also reinitialize
void KalmanFilter::reinitialize(Eigen::Vector2f acc_std, 
				Eigen::Vector2f measur_std,
				float delta_t,
				Eigen::Vector4f x_k1,
				Eigen::Matrix4f init_cov
				/*=Eigen::Matrix4f::Zero()*/)
{
  x_k_n_ =  x_k1; //Initialize to 
  P_k1_ = init_cov;

  H_.fill(0.);
  H_(0,0) = 1.;
  H_(1,1) = 1.;

  R_.fill(0.);
  R_(0,0) = pow(measur_std(0),2);
  R_(1,1) = pow(measur_std(1),2);

  sigma_acc_ = Eigen::Matrix2f::Zero();
  sigma_acc_(0,0) = pow(acc_std(0),2);
  sigma_acc_(1,1) = pow(acc_std(1),2);

  delta_t_ = delta_t;
  delta_change();
}

void KalmanFilter::delta_change()
{
    //Recompute A
    A_ = Eigen::Matrix4f::Identity();
    A_(0,2) = delta_t_;
    A_(1,3) = delta_t_;
    
    //Recompute Q
    float dt2 = (pow(delta_t_,2))/2;
    Eigen::Matrix<float, 4, 2>  G;
      G << dt2, 0.,
      0., dt2,
      delta_t_, 0.,
      0., delta_t_;
    
    Q_ = G * sigma_acc_ * G.transpose();  
}

void KalmanFilter::estimate(Eigen::Vector2f obs, float delta_t, Eigen::Vector4f &est)
{
  //previous estimate is k-1th estimate now
  x_k1_ = x_k_n_;
  predict(delta_t);
  correct(obs);

  est = x_k_n_;
  return;
}
