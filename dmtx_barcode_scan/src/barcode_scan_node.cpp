#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

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

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  ROS_INFO("IMG");
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
  // cv::Mat* img_mat = &cv_ptr->image;
  try
  {
    imshow("view", cv_ptr->image);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }

  ////////////////////////////////////////////////////////////////
  DmtxImage      *img;
  DmtxDecode     *dec;
  DmtxRegion     *reg;
  DmtxMessage    *dmtx_msg;

  img = dmtxImageCreate(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, 
                        DmtxPack24bppBGR);
  assert(img != NULL);
ROS_INFO("A %d %d", img->width, img->height);

  dec = dmtxDecodeCreate(img, 1);
  assert(dec != NULL);
ROS_INFO("B");

  reg = dmtxRegionFindNext(dec, NULL);
ROS_INFO("C");
  if(reg != NULL) {
     dmtx_msg = dmtxDecodeMatrixRegion(dec, reg, DmtxUndefined);
ROS_INFO("D");
     if(dmtx_msg != NULL) {
ROS_INFO("E");
        const char* output = reinterpret_cast<const char*>(dmtx_msg->output);
        ROS_INFO("Message: %s", output);
        dmtxMessageDestroy(&dmtx_msg);
     } else {
        ROS_INFO("Message not found");
     }
     dmtxRegionDestroy(&reg);
  }

ROS_INFO("F");
  dmtxDecodeDestroy(&dec);
  dmtxImageDestroy(&img);
ROS_INFO("G");
}

int main(int argc, char **argv)
{
  // return blah();
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cvNamedWindow("view");
  cvStartWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("/camera/image_raw", 1, imageCallback);
  ros::spin();
  cvDestroyWindow("view");
}
