#include <bmw_percep/tracker_3D.hpp>

int main(int argc, char** argv)
{
  cout << "Compiling this? " << endl;
  ros::init(argc, argv, "sample_tracker");
  ros::NodeHandle nh_priv("~");
  Tracker3d tracker(nh_priv);
  return 0;
}
