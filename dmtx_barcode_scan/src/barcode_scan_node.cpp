#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <dmtx_barcode_scan/Scan.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <dmtx.h>

/*
int
blah()
{
   size_t          width, height, bytesPerPixel;
   unsigned char   str[] = "30Q324343430794<OQQ";
   unsigned char  *pxl;
   DmtxEncode     *enc;
   DmtxImage      *img;
   DmtxDecode     *dec;
   DmtxRegion     *reg;
   DmtxMessage    *msg;

   fprintf(stdout, "input:  \"%s\"\n", str);

   // 1) ENCODE a new Data Matrix barcode image (in memory only) 

   enc = dmtxEncodeCreate();
   assert(enc != NULL);
   dmtxEncodeDataMatrix(enc, std::strlen(reinterpret_cast<const char*>(str)), str);

   // 2) COPY the new image data before releasing encoding memory 

   width = dmtxImageGetProp(enc->image, DmtxPropWidth);
   height = dmtxImageGetProp(enc->image, DmtxPropHeight);
   bytesPerPixel = dmtxImageGetProp(enc->image, DmtxPropBytesPerPixel);

   pxl = (unsigned char *)malloc(width * height * bytesPerPixel);
   assert(pxl != NULL);
   memcpy(pxl, enc->image->pxl, width * height * bytesPerPixel);

   dmtxEncodeDestroy(&enc);

   // 3) DECODE the Data Matrix barcode from the copied image 

   img = dmtxImageCreate(pxl, width, height, DmtxPack24bppRGB);
   assert(img != NULL);

   dec = dmtxDecodeCreate(img, 1);
   assert(dec != NULL);

   reg = dmtxRegionFindNext(dec, NULL);
   if(reg != NULL) {
      msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
      if(msg != NULL) {
         fputs("output: \"", stdout);
         fwrite(msg->output, sizeof(unsigned char), msg->outputIdx, stdout);
         fputs("\"\n", stdout);
         dmtxMessageDestroy(&msg);
      }
      dmtxRegionDestroy(&reg);
   }

   dmtxDecodeDestroy(&dec);
   dmtxImageDestroy(&img);
   free(pxl);

   exit(0);
}
*/

void imageCallback(const sensor_msgs::ImageConstPtr& msg);
//GLOBALS
ros::Publisher pub_scan;

int main(int argc, char **argv)
{
  // return blah();
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cvNamedWindow("view");
  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/scan_barcode_cam/image_raw", 1, imageCallback);
  pub_scan = nh.advertise<dmtx_barcode_scan::Scan>("scan/", 1);
  ros::spin();
  cvDestroyWindow("view");
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  std::string expect_msg = "ASIF";

  ROS_INFO("IMG");
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, "mono8");
  // cv::Mat* img_mat = &cv_ptr->image;
  try
  {
    imshow("view", cv_ptr->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
  }

  ////////////////////////////////////////////////////////////////
  DmtxImage      *img;
  DmtxDecode     *dec;
  DmtxRegion     *reg;
  DmtxMessage    *dmtx_msg;

  dmtx_barcode_scan::Scan smsg;
  smsg.found = false;
  std::cout << "Scan Message" << smsg << std::endl;

  img = dmtxImageCreate(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, 
                        DmtxPack8bppK);
  assert(img != NULL);
  ROS_INFO("A %d %d", img->width, img->height);

  dec = dmtxDecodeCreate(img, 1);
  assert(dec != NULL);
  ROS_INFO("B");

  reg = dmtxRegionFindNext(dec, NULL);
  while(reg!=NULL){
    ROS_INFO("C");

    dmtx_msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
    ROS_INFO("D");
    if(dmtx_msg != NULL) {

      std::string str_dmtx((char*)reinterpret_cast<char*>(dmtx_msg->output));

      if(str_dmtx == expect_msg){
      ROS_INFO("E");
      smsg.found = true;

      smsg.tag = str_dmtx;
      std::cout << "Scan Message. True?" << smsg << std::endl;
   
      char c = cv::waitKey(500);
      const char* output = reinterpret_cast<const char*>(dmtx_msg->output);
      ROS_INFO("Message: %s", output);
      dmtxMessageDestroy(&dmtx_msg);
      break;
      }
    } else {
      ROS_INFO("Message not found");
    }
    dmtxRegionDestroy(&reg);
  }
  
  ROS_INFO("F");
  dmtxDecodeDestroy(&dec);
  dmtxImageDestroy(&img);
  ROS_INFO("G");

  std::cout << "Scan Message before publish." << smsg << std::endl;
  pub_scan.publish(smsg);

}
