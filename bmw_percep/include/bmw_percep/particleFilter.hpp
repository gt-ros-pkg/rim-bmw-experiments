#include<iostream>
#include<vector>

/**
   Class *definitions* for 2D particle filter
**/

using namespace std;

typedef vector<vector< double> > State;  // x, y, vx, vy and weights

class particleFilter2D{
public:
  particleFilter2D();
  
  void estimate();

private:
  State cur_state;
  State prev_state;
  int n_particles;
};
