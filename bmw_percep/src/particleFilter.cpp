#include<bmw_percep/particleFilter.hpp>

/**
   Class *implementation* for 2D particle filter
**/

using namespace std;

#define PI 22./7.

double gaussian_at_point(double mean, double sigma, double point);

particleFilter2D::particleFilter2D(cv::Point2f obs_1, cv::Point2f obs_2, 
				   double delta_t,
				   int no_part /*=1000*/, double std_acc/*=1.*/,
				   double std_pos/*=1.5*/,
				   double std_vel/*=2.0*/, 
				   double eff_part_ratio/*=.2*/)
{
  acc_std = std_acc;
  delta = delta_t;
  cv::Mat part_mat = cv::Mat::ones(no_part, 5, CV_64FC1); // each row - [x, y, vx, vy, w]

  part_mat.col(0) *= obs_2.x; part_mat.col(1) *= obs_2.y;
  cv::Point2f init_vel = obs_2 - obs_1;
  part_mat.col(2) *= init_vel.x; part_mat.col(3) *= init_vel.y;
  double init_w = 1.0/static_cast<double>(no_part);

  // Randomly perturb particles
  cv::Mat pertur_mat = cv::Mat::zeros(part_mat.size(), part_mat.depth());
  //perturb positions
  cv::randn(pertur_mat.col(0), cv::Mat::zeros(1,1,CV_64FC1), 
	    std_pos * cv::Mat::ones(1,1,CV_64FC1));
  cv::randn(pertur_mat.col(1), cv::Mat::zeros(1,1,CV_64FC1), 
	    std_pos * cv::Mat::ones(1,1,CV_64FC1));
  //perturb velocities
  cv::randn(pertur_mat.col(2), cv::Mat::zeros(1,1,CV_64FC1), 
	    std_vel * cv::Mat::ones(1,1,CV_64FC1));
  cv::randn(pertur_mat.col(3), cv::Mat::zeros(1,1,CV_64FC1), 
	    std_vel * cv::Mat::ones(1,1,CV_64FC1));
  
  part_mat += pertur_mat;

  cur_state = part_mat;
  
  //normalize
  cur_state.col(cur_state.cols-1) *= (1.0/static_cast<double>(n_particles));
  
}

void particleFilter2D::reinitialize(cv::Point2f obs_1, cv::Point2f obs_2, 
				   double delta_t,
				   int no_part /*=1000*/, double std_acc/*=10.0*/,
				   double std_pos/*=1.5*/,
				   double std_vel/*=2.0*/, 
				   double eff_part_ratio/*=.2*/)
{
  //debug
  //cout << "start iniitialize " << endl;
  effective_part_thresh = eff_part_ratio;
  acc_std = std_acc;
  delta = delta_t;
  n_particles = no_part;

  cv::Mat part_mat = 
    cv::Mat::ones(no_part, 5, CV_64FC1); // each row - [x, y, vx, vy, w]

  part_mat.col(0) *= obs_2.x; part_mat.col(1) *= obs_2.y;
  cv::Point2f init_vel = obs_2 - obs_1;
  part_mat.col(2) *= init_vel.x; part_mat.col(3) *= init_vel.y;
  double init_w = 1.0/static_cast<double>(no_part);

  // Randomly perturb particles
  cv::Mat pertur_mat = cv::Mat::zeros(part_mat.size(), part_mat.depth());
  //perturb positions
  cv::randn(pertur_mat.col(0), cv::Mat::zeros(1,1,CV_64FC1), 
	    (sqrt(2) * std_pos) * cv::Mat::ones(1,1,CV_64FC1));
  cv::randn(pertur_mat.col(1), cv::Mat::zeros(1,1,CV_64FC1), 
	    (sqrt(2) * std_pos) * cv::Mat::ones(1,1,CV_64FC1));
  //perturb velocities
  cv::randn(pertur_mat.col(2), cv::Mat::zeros(1,1,CV_64FC1), 
	    (sqrt(2) * std_vel) * cv::Mat::ones(1,1,CV_64FC1));
  cv::randn(pertur_mat.col(3), cv::Mat::zeros(1,1,CV_64FC1), 
	    (sqrt(2) * std_vel) * cv::Mat::ones(1,1,CV_64FC1));
  
  part_mat += pertur_mat;
  
  part_mat.copyTo(cur_state);

  //normalize
  cur_state.col(cur_state.cols-1) *= (1.0/static_cast<double>(n_particles));
  
  //cout << cur_state << endl;
  
  //debug
  //cout << "done iniitialize " << endl;
}

particleFilter2D::particleFilter2D()
{
  cout << "\nDeprecated constructor, use others.." << endl;
}

void particleFilter2D::estimate(const cv::Point2f obs, const cv::Point2f obs_vel, 
	      cv::Point2f &pos, cv::Point2f &vel)
{
  // weigh them according to observation
  reweigh(obs, obs_vel);

  // get effective number of particles
  cv::Mat prev_weights; cur_state.col(cur_state.cols-1).copyTo(prev_weights);
  cv::Mat temp = 1.0/(prev_weights.t() * prev_weights);
  //debug
  //cout << "Current State" << cur_state << endl ;
  //cout << "Previous weights" << prev_weights << endl;
  double N_eff = temp.at<double>(0,0);
  
  if ((N_eff/static_cast<double>(n_particles)) < effective_part_thresh){
    //debug
    cout << "Resample? " << endl;
    resample_particles();
  }


  //debug
  // cout << "Re-weighed " << endl;
  //cout << "\nBefore mean: " << cur_state << endl;

  // get point estimate
  cv::Mat particle_estimate;
  weighted_mean(particle_estimate);

  //debug
  // cout << "Got mean?" << endl;

  //TODO: velocity to be m/s not m/frame
  pos.x = particle_estimate.at<double>(0,0);
  pos.y = particle_estimate.at<double>(0,1);
  vel.x = particle_estimate.at<double>(0,2)/30.0;
  vel.y = particle_estimate.at<double>(0,3)/30.0;
  
  // cout << "done estimation" << endl;
}

 
void particleFilter2D::estimate(const cv::Point2f obs, cv::Point2f &pos, 
				cv::Point2f &vel)
{
  //debug
  // cout << "Got to estimation" << endl;

  // weigh them according to observation
  reweigh(obs);

  // get effective number of particles
  cv::Mat prev_weights; cur_state.col(cur_state.cols-1).copyTo(prev_weights);
  cv::Mat temp = 1.0/(prev_weights.t() * prev_weights);
  //debug
  //cout << "Current State" << cur_state << endl ;
  //cout << "Previous weights" << prev_weights << endl;
  double N_eff = temp.at<double>(0,0);
  
  if ((N_eff/static_cast<double>(n_particles)) < effective_part_thresh){
    //debug
    cout << "Resample? " << endl;
    resample_particles();
  }


  //debug
  // cout << "Re-weighed " << endl;
  //cout << "\nBefore mean: " << cur_state << endl;

  // get point estimate
  cv::Mat particle_estimate;
  weighted_mean(particle_estimate);

  //debug
  // cout << "Got mean?" << endl;

  //TODO: velocity to be m/s not m/frame
  pos.x = particle_estimate.at<double>(0,0);
  pos.y = particle_estimate.at<double>(0,1);
  vel.x = particle_estimate.at<double>(0,2)/30.0;
  vel.y = particle_estimate.at<double>(0,3)/30.0;
  
  // cout << "done estimation" << endl;
}

void particleFilter2D::prop_part()
{
  // propogate them particles
  //positions
  cv::Mat pertur_mat = cv::Mat::zeros(cur_state.size(), cur_state.depth());
  // positions process noise
  cv::randn(pertur_mat.col(0), cv::Mat::zeros(1,1,CV_64FC1), 
	    acc_std * cv::Mat::ones(1,1,CV_64FC1));
  cv::randn(pertur_mat.col(1), cv::Mat::zeros(1,1,CV_64FC1), 
	    acc_std * cv::Mat::ones(1,1,CV_64FC1));
  // velocity process noise
  cv::randn(pertur_mat.col(2), cv::Mat::zeros(1,1,CV_64FC1), 
	    acc_std * cv::Mat::ones(1,1,CV_64FC1));
  cv::randn(pertur_mat.col(3), cv::Mat::zeros(1,1,CV_64FC1), 
	    acc_std * cv::Mat::ones(1,1,CV_64FC1));
  // pos = pos + vel*t + acc.* t^2
  cur_state.col(0) += cur_state.col(2) * delta + pertur_mat.col(0) * pow(delta,2.0);
  cur_state.col(1) += cur_state.col(3) * delta + pertur_mat.col(1) * pow(delta,2.0);
  // velocity = velocity + acc.* t^2 
  cur_state.col(2) += pertur_mat.col(2) * delta;
  cur_state.col(3) += pertur_mat.col(3) * delta;

}

// void particleFilter2D::prop_estimate(const cv::Point2f obs, cv::Point2f &pos, 
// 				cv::Point2f &vel)
// {
//   //debug
//   cout << "Got to estimation" << endl;

//   // get effective number of particles
//   cv::Mat prev_weights; cur_state.col(cur_state.cols-1).copyTo(prev_weights);
//   cv::Mat temp = 1.0/(prev_weights.t() * prev_weights);
//   //debug
//   //cout << "Current State" << cur_state << endl ;
//   //cout << "Previous weights" << prev_weights << endl;
//   double N_eff = temp.at<double>(0,0);
  
//   if ((N_eff/static_cast<double>(n_particles)) < effective_part_thresh){
//     //debug
//     cout << "Resample? " << endl;
//     resample_particles();
//   }
//   // propogate them particles
//   //positions
//   cv::Mat pertur_mat = cv::Mat::zeros(cur_state.size(), cur_state.depth());
//   // positions process noise
//   cv::randn(pertur_mat.col(0), cv::Mat::zeros(1,1,CV_64FC1), 
// 	    acc_std * cv::Mat::ones(1,1,CV_64FC1));
//   cv::randn(pertur_mat.col(1), cv::Mat::zeros(1,1,CV_64FC1), 
// 	    acc_std * cv::Mat::ones(1,1,CV_64FC1));
//   // velocity process noise
//   cv::randn(pertur_mat.col(2), cv::Mat::zeros(1,1,CV_64FC1), 
// 	    acc_std * cv::Mat::ones(1,1,CV_64FC1));
//   cv::randn(pertur_mat.col(3), cv::Mat::zeros(1,1,CV_64FC1), 
// 	    acc_std * cv::Mat::ones(1,1,CV_64FC1));
//   // pos = pos + vel*t + acc.* t^2
//   cur_state.col(0) += cur_state.col(2) * delta + pertur_mat.col(0) * pow(delta,2.0);
//   cur_state.col(1) += cur_state.col(3) * delta + pertur_mat.col(1) * pow(delta,2.0);
//   // velocity = velocity + acc.* t^2 
//   cur_state.col(2) += pertur_mat.col(2) * delta;
//   cur_state.col(3) += pertur_mat.col(3) * delta;
//   //debug
//   //cout << "Propogated.. " << endl;

//   // weigh them according to observation
//   reweigh(obs);
//   //debug
//   cout << "Re-weighed " << endl;
//   //cout << "\nBefore mean: " << cur_state << endl;
//   // get point estimate
//   cv::Mat particle_estimate;
//   weighted_mean(particle_estimate);
//   //debug
//   cout << "Got mean?" << endl;

//   pos.x = particle_estimate.at<double>(0,0);
//   pos.y = particle_estimate.at<double>(0,1);
//   vel.x = particle_estimate.at<double>(0,2);
//   vel.y = particle_estimate.at<double>(0,3);

//   // //change vel to m/frame
//   // vel.x /= 30.0;
//   // vel.y /= 30.0;

//   cout << "done estimation" << endl;
// }

void particleFilter2D::weighted_mean(cv::Mat& p_est)
{
  cv::Mat temp_p = cv::Mat::zeros(1, cur_state.cols, cur_state.depth());
  //debug
  cout << "Set temp" << endl;

  for (int i=0; i<n_particles; i++){
    double cur_w = cur_state.at<double>(i, cur_state.cols-1);
    temp_p +=  cur_w * cur_state.row(i);
  }
  
  cout << "Temp - p" << temp_p << endl;
  //debug
  cout << "Weird multiplication done?    " << endl;

  p_est = cv::Mat::zeros(temp_p.rows, temp_p.cols-1, temp_p.depth());

  for(int i=0; i<p_est.cols; i++){
    p_est.at<double>(0,i) = temp_p.at<double>(0,i);
  }
  cout << "P--est " << p_est << endl;
}

void particleFilter2D::reweigh(cv::Point2f obs,   
			       cv::Point2f obs_v, 
			       double sigma/*=1.0*/,
			       double sigma_v/*=2.0*/)
{
  //TODO: Think about numerical stability, log-likelihoods perhaps
  double epsilon = 1.e-15; // because couldn't get the one from numerical limits
  cv::Mat new_weights(cur_state.rows, 1, cur_state.depth());
  double sum_wts = 0.0, max_wt=0.0;

  for(int i=0; i<n_particles; i++)
    {
      double* state_vec = cur_state.ptr<double> (i);
      double dist = sqrt(pow(state_vec[0]-obs.x,2.0) +pow(state_vec[1]-obs.y,2.0));
      double similarity = gaussian_at_point(0.0, sigma, dist);
      
      // //in case too far away, I still want to keep the particle
      // if (similarity < 10.0* epsilon)
      // 	similarity = 10.0* epsilon;
      
      //in case too far away, I want to get rid of the particle
      if (similarity < 10*epsilon)
	similarity = 0.0;

      //set weight
      state_vec[cur_state.cols-1] *= similarity;
      
      if (state_vec[cur_state.cols-1] < 10*epsilon)
	state_vec[cur_state.cols-1] = 0.0;
      
      sum_wts += state_vec[cur_state.cols-1];
    }
  
  //debug
  //cout << "new weights check." << endl;

  //check for numerical stability
  if (sum_wts>1){
    for(int i=0; i<n_particles; i++){
      double* state_vec = cur_state.ptr<double> (i);
      double similarity = state_vec[cur_state.cols-1];
      if (similarity < 10*sum_wts*epsilon){
	//similarity = 10*sum_wts* epsilon;
	state_vec[cur_state.cols-1] = 0.0;
      }
    }
  }
  if (sum_wts<epsilon){
    //weigh everyone equally
    cur_state.col(cur_state.cols-1) = cv::Mat::ones(n_particles, 1, 
						    cur_state.depth());
  }
  //debug
  //cout << "Go to normalize.. " << endl;
  
  //normalize weights
  cur_state.col(cur_state.cols-1) *= (1.0/sum_wts);
}

void particleFilter2D::reweigh(cv::Point2f obs, double sigma/*=0.5*/)
{
  //TODO: Think about numerical stability, log-likelihoods perhaps
  double epsilon = 1.e-15; // because couldn't get the one from numerical limits
  cv::Mat new_weights(cur_state.rows, 1, cur_state.depth());
  double sum_wts = 0.0, max_wt=0.0;

  for(int i=0; i<n_particles; i++)
    {
      double* state_vec = cur_state.ptr<double> (i);
      double dist = sqrt(pow(state_vec[0]-obs.x,2.0) +pow(state_vec[1]-obs.y,2.0));
      double similarity = gaussian_at_point(0.0, sigma, dist);
      
      // //in case too far away, I still want to keep the particle
      // if (similarity < 10.0* epsilon)
      // 	similarity = 10.0* epsilon;
      
      //in case too far away, I want to get rid of the particle
      if (similarity < 10*epsilon)
	similarity = 0.0;

      //set weight
      state_vec[cur_state.cols-1] *= similarity;
      
      if (state_vec[cur_state.cols-1] < 10*epsilon)
	state_vec[cur_state.cols-1] = 0.0;
      
      sum_wts += state_vec[cur_state.cols-1];
    }
  
  //debug
  //cout << "new weights check." << endl;

  //check for numerical stability
  if (sum_wts>1){
    for(int i=0; i<n_particles; i++){
      double* state_vec = cur_state.ptr<double> (i);
      double similarity = state_vec[cur_state.cols-1];
      if (similarity < 10*sum_wts*epsilon){
	//similarity = 10*sum_wts* epsilon;
	state_vec[cur_state.cols-1] = 0.0;
      }
    }
  }
  if (sum_wts<epsilon){
    //weigh everyone equally
    cur_state.col(cur_state.cols-1) = cv::Mat::ones(n_particles, 1, 
						    cur_state.depth());
  }
  //debug
  //cout << "Go to normalize.. " << endl;
  
  //normalize weights
  cur_state.col(cur_state.cols-1) *= (1.0/sum_wts);
}

double gaussian_at_point(double mean, double sigma, double point)
{
  return (1.0/sqrt(2*(PI)*sigma*sigma)) * exp(-pow((point-mean),2)/(2.*sigma*sigma));
}

//sampling a discrete distribution - copied from :
//http://www.boost.org/doc/libs/1_46_1/doc/html/boost_random/tutorial.html#boost_random.tutorial.generating_integers_with_different_probabilities
int particleFilter2D::roll_weighted_die(vector<double> disc_dist) 
{
  std::vector<double> cumulative;
  std::partial_sum(disc_dist.begin(), disc_dist.end(),
		   std::back_inserter(cumulative));
  boost::uniform_real<> dist(0, cumulative.back());
  boost::variate_generator<boost::mt19937&, boost::uniform_real<> > die(gen, dist);
  return (std::lower_bound(cumulative.begin(), cumulative.end(), die()) 
	  - cumulative.begin()) ;
}

void particleFilter2D::resample_particles()
{
  vector<double> prev_weights;
  cv::Mat temp_state(cur_state.size(), cur_state.depth());
  
  prev_weights.clear();
  for (int i=0; i<n_particles; i++){
    prev_weights.push_back(cur_state.at<double>(i, cur_state.cols-1));
  }
    
  for (int i=0; i<n_particles; i++){
    int chosen_one = roll_weighted_die(prev_weights);
    cur_state.row(chosen_one).copyTo(temp_state.row(i));
  }

  //make all the weights equal & normalize
  temp_state.col(temp_state.cols-1) = (1.0/static_cast<double>(n_particles)) *
    cv::Mat::ones(n_particles, 1, temp_state.depth());
  
  //copy over to the class variable
  cur_state = temp_state;
}
