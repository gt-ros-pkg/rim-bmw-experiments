#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

/**
   Class definition for Color Histogram that 
   
   by Shray Bansal
**/

using namespace std;

class ColorHistogram{

public:
  ColorHistogram(){}
  
  // a_count = alpha count meaning the intial count in each bin
  ColorHistogram(int bins, int a_count=1);

  // read the histogram from a file
  ColorHistogram(int bins, string path, int a_count=1);

  void addToObj(const cv::Mat& frame);
  void addToBackground(const cv::Mat& frame);
  void normalize();
  void writeToFile(string path);
  void readFromFile(string path);
  void displayHists();
  void testImg(const cv::Mat &im, cv::Mat &fore);

private:
  void augmentHist(cv::Mat& hist, const cv::Mat& bgr_frame);
  cv::Point getIndex(uchar h, uchar s);
  int _bins; // bins in each of the H and S dimensions
  float _h_size, _s_size;
  cv::Mat object_histogram; // hist for object colors
  cv::Mat background_histogram; // hist for back. colors
  double _thresh;
};
