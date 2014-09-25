#include <iostream>
#include <fstream>
#include <iterator>
#include <sstream>
#include <vector>
#include <string>

#include <boost/lexical_cast.hpp>

#include <Eigen/Core>

#include <bmw_percep/kalmanFilterAcc.hpp>

/** 
    Reads in a file with 2D points, and time-diffs. Kalman
    Filters them with different params. Stores the points with
    the filtered estimates, velocities and accelerations in as many
    files as the parameters
**/

//Param being changed is jerk
#define MIN_JERK .005
#define MAX_JERK 2.
#define JERK_STEP .005

using namespace std;

void run_kf(string kf_fname, float jerk, 
	    vector<Eigen::Vector2f> observations, vector<float> time_diffs);

//copied from: http://stackoverflow.com/questions/1120140/how-can-i-read-and-parse-csv-files-in-c
class CSVRow
{
public:
  string const& operator[](size_t index) const
  {
    return m_data[index];
  }
  size_t size() const
  {
    return m_data.size();
  }
  void readNextRow(istream& str)
  {
    string         line;
    getline(str,line);

    stringstream   lineStream(line);
    string         cell;

    m_data.clear();
    while(getline(lineStream,cell,','))
      {
	m_data.push_back(cell);
      }
  }
private:
  vector<string>    m_data;
};

istream& operator>>(istream& str,CSVRow& data)
{
  data.readNextRow(str);
  return str;
}   

//MAIN
int main(int argc, char** argv)
{

  string path="data/kf_params/normal2/";
  string filename = path+"observations.csv";
  ifstream file(filename.c_str());
  
  vector<Eigen::Vector2f> observations;
  vector<float> time_diffs;
  CSVRow row;
  while(file >> row){
    observations.push_back(Eigen::Vector2f(boost::lexical_cast<float>(row[0]),
					   boost::lexical_cast<float>(row[1])));
    time_diffs.push_back(boost::lexical_cast<float>(row[2]));
  }

  string ext=".csv";
  float jerk;
  string kf_fname;
  size_t num_iter = ceil((MAX_JERK - MIN_JERK)/JERK_STEP);

  for(size_t i=0; i<num_iter; ++i){
    jerk = MIN_JERK + static_cast<float>(i) * JERK_STEP;
    kf_fname = path + boost::lexical_cast<string>(i) + ext;
    run_kf(kf_fname, jerk, observations, time_diffs);
    cout << (static_cast<float>(i+1)/static_cast<float>(num_iter))*100. << "% Done!" << endl;
  }

  return 0;

}

void run_kf(string kf_fname, float jerk, 
	    vector<Eigen::Vector2f> observations, vector<float> time_diffs)
{
  ofstream kf_file(kf_fname.c_str());

  //Create filter
  KalmanFilterAcc kf;

  bool kf_initialized=false;

  Eigen::Vector2f jerk_std(jerk,jerk);
  Eigen::Vector2f measur_std(0.05,0.05);
  float delta_t = 1./15.;
  Eigen::Matrix<float, 6, 1> x_k1;
  x_k1.fill(0.);
  Eigen::Matrix<float, 6, 6> init_cov 
    = 10000*Eigen::MatrixXf::Identity(6,6);

  Eigen::Matrix<float, 6, 1> kf_est;
  vector<Eigen::Matrix<float, 6, 1> > kf_states;

  //filter
  for(size_t i=0; i<observations.size(); ++i){
    
    Eigen::Vector2f cur_obs(observations[i](0), observations[i](1));
    
    if (!kf_initialized){ // in case filter not initialized
      x_k1(0,0)=cur_obs(0); x_k1(1,0)=cur_obs(1);
      kf.reinitialize(jerk_std, measur_std, 
		      delta_t, x_k1, init_cov);
      kf_initialized = true;
    }
    else{ // I take delta-ts after the first frame.
      delta_t = time_diffs[i];
    }

    kf.estimate(cur_obs, delta_t, kf_est);
    kf_states.push_back(kf_est);
    
    //write to file
    kf_file << cur_obs(0) << ',' << cur_obs(1) << ',' << kf_est(0) << ',' 
	    << kf_est(1) << ',' << kf_est(2) << ',' << kf_est(3) << ',' 
	    << kf_est(4) << ',' << kf_est(5) << ',' << delta_t << endl;
  }

  kf_file.close();

}
