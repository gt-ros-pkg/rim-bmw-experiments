#ifndef KALMAN_FILTER_ACC
#define KALMAN_FILTER_ACC

#include <iostream>
#include <Eigen/Dense>

/**
   Class *definitions* for Kalman Filter with state
   (x,y,vx,vy) and measurement (x,y)
**/

typedef Eigen::Matrix<float, 6, 1> State;
using namespace std;


class KalmanFilterAcc{

public:
  //TODO: What parameters go in to initialize?
  KalmanFilterAcc();

  //if delta-t not known pass 0
  void predict(float delta_t);

  //incase only prediction is required and not update of everything
  void predict(State &s,Eigen::Matrix<float, 6, 6> &P, float delta_t);
  
  void correct(Eigen::Vector2f z_k);
  
  void reinitialize(Eigen::Vector2f jerk_std,
		    Eigen::Vector2f measur_std,
		    float delta_t,
		    Eigen::Matrix<float, 6,1> x_k1,
		    Eigen::Matrix<float, 6, 6> init_cov = 
		    Eigen::Matrix<float, 6, 6>::Zero());
  
  //estimate new state vector given observation
  void estimate(Eigen::Vector2f obs, float delta_t, Eigen::Matrix<float, 6, 1> &est);
  //returns the error of the observation w.r.t the current filter
  float estimate_check(Eigen::Vector2f obs, float delta_t);
  
  //get current state
  State get_state();

private:
  Eigen::Matrix<float, 6,1> x_k_p_, x_k_n_; //previous and next Xk s
  Eigen::Matrix<float, 6,1> x_k1_; // Xk-1
  Eigen::Vector2f z_k_;
  Eigen::Matrix<float, 6, 6> A_, Q_, P_k_p_, P_k_n_, P_k1_;
  Eigen::Matrix2f R_, sigma_jerk_;
  Eigen::Matrix<float, 2, 6> H_;
  Eigen::Matrix<float, 6, 2> K_;
  
  float delta_t_;

  //Modifies the variables associated with change in delta-t
  void delta_change();
};
#endif
