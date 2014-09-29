#include <dmtx_barcode_scan/barcodeScan.hpp>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "bacodes");
  ros::NodeHandle nh;
  
  BarcodeScan bscan(nh);
  
  bscan.find_tag("idrive", 10.);
  
  ros::spin();
  return 0;
}


