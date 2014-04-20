#include<iostream>
#include<vector>
#include<opencv2/opencv.hpp>
#include<boost/random/uniform_real.hpp>
#include<boost/random/mersenne_twister.hpp>
#include<boost/random/variate_generator.hpp>
#include<numeric>
#include<boost/random/normal_distribution.hpp>
#include<limits>
/**
   Class *definitions* for 2D particle filter
   -- has a bit of unnecessary dependence on OpenCV - to get rid of later
**/

// TODO: Have a notion of maximums and some way of keeping estimates within it

using namespace std;

typedef vector<vector< double> > State;  // x, y, vx, vy and weights

class particleFilter2D{
public:

  //deprecated constructor
  particleFilter2D();

  // initialize with two observed locations, this enables particles to
  // be initialized using both position and velocity measures
  particleFilter2D(cv::Point2f obs_1, cv::Point2f obs_2, double delta_t=(1.0/30.0), 
		   int no_part =1000, double std_acc=10.0,
		   double std_pos=1.5, double std_vel=2.0, double eff_part_ratio=0.2);
  
  //initialize particles all over again
  void reinitialize(cv::Point2f obs_1, cv::Point2f obs_2, double delta_t=(1.0/30.0), 
		   int no_part =1000, double std_acc=10.0,
		   double std_pos=1.5, double std_vel=2.0, double eff_part_ratio=0.2);
  
  //propogate particles one step in time
  void prop_part();

  //point-estimate of the state given an observation of position
  void estimate(const cv::Point2f obs, cv::Point2f &pos, cv::Point2f &vel);

  //point-estimate of the state given an observation of position & velocity
  void estimate(const cv::Point2f obs, const cv::Point2f obs_vel, 
		cv::Point2f &pos, cv::Point2f &vel);

  // //point-estimate of the state given an observation of position
  // void prop_estimate(const cv::Point2f obs, cv::Point2f &pos, cv::Point2f &vel);

private:
  void reweigh(cv::Point2f obs, double sigma=1.0); //reweigh particles
						   //according to
						   //obsevation
  void reweigh(cv::Point2f obs,   
	       cv::Point2f obs_v, 
	       double sigma=1.0,
	       double sigma_v=2.0); //reweigh particles according to
				  //obsevation of position & velocity
  void resample_particles(); //resample particles from weight
			     //distribution
  int roll_weighted_die(vector<double> disc_dist); // sample from
						   // discrete
						   // distribution
  void weighted_mean(cv::Mat& p_est); // computes weighted mean of
					// particles
  cv::Mat cur_state; // current state of particles 
  State prev_state;
  double acc_std; // standard-dev for acceleration
  int n_particles; // no. of particles
  double delta; // time-step
  double effective_part_thresh; // if the ratio of effective to
				// ineffictive particles goes below
				// threshold then resample
  boost::mt19937 gen;
};
