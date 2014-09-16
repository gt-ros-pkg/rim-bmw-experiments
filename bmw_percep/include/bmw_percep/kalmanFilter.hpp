#include <iostream>
#include <Eigen/Dense>

/**
   Class *definitions* for Kalman Filter with state
   (x,y,vx,vy) and measurement (x,y)
**/

using namespace std;


class KalmanFilter{

public:
  //TODO: What parameters go in to initialize?
  KalmanFilter();

  //if delta-t not known pass 0
  void predict(float delta_t);
  
  void correct(Eigen::Vector2f z_k);

private:
  Eigen::Vector4f x_k_p_, x_k_n_; //previous and next Xk s
  Eigen::Vector4f x_k1_; // Xk-1
  Eigen::Vector2f z_k_;
  Eigen::Matrix4f A_, Q_, P_k_p_, P_k_n_, P_k1_;
  Eigen::Matrix2f R_;
  Eigen::Matrix<float, 2, 4> H_;
  Eigen::Matrix<float, 4, 2> K_;
  
  float delta_t_;
};
