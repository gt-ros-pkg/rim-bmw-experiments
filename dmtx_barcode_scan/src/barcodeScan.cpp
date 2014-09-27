#include <dmtx_barcode_scan/barcodeScan.hpp>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "bacodes");
  ros::NodeHandle nh;
  
  BarcodeScan bscan(nh);

  ros::spin();
  return 0;
}


