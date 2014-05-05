#include <bmw_percep/colorHistogram.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

/**
   Class implementations for Color Histogram 
   
   by Shray Bansal
**/

double THRESH = 1.0;

// a_count = alpha count meaning the intial count in each bin
ColorHistogram::ColorHistogram(int bins, int a_count/*=1*/)
{

  _thresh = THRESH;

  float h_range = 181.0;
  float s_range = 256.0;


  _bins_s = bins;
  _bins_h = 181;

  object_histogram = static_cast<double> (a_count) * 
    cv::Mat::ones(_bins_h, _bins_s, CV_64FC1);

  background_histogram = static_cast<double> (a_count) * 
    cv::Mat::ones(_bins_h, _bins_s, CV_64FC1);
  
  _h_size = static_cast<float>(h_range/_bins_h);
  _s_size = static_cast<float>(s_range/_bins_s);
  
  //debug
  cout << "Bins = " << _bins_h << "\t" << _bins_s << endl;
  cout << "H size = " << _h_size << "\t S size = " << _s_size << endl;
  cout << "Threshold = " << _thresh << endl;
  cout << "(0,0) Index: " << getIndex((uchar)0,(uchar)0) << endl;
  cout << "(20,50) Index: " << getIndex((uchar)20,(uchar)50) << endl;
  cout << "(180,255) Index: " << getIndex((uchar)180,(uchar)255) << endl;
  return;
}

void ColorHistogram::addToObj(const cv::Mat& frame)
{augmentHist(object_histogram, frame);}

void ColorHistogram::addToBackground(const cv::Mat& frame)
{augmentHist(background_histogram, frame);}  

void ColorHistogram::augmentHist(cv::Mat& hist, const cv::Mat& bgr_frame)
{

  //convert frame to HSV
  cv::Mat converted;
  cv::cvtColor(bgr_frame, converted, CV_BGR2HSV);
  cv::Point cur_ind;

  for (int r=0; r<converted.rows; r++){
    cv::Vec3b* conv_r = converted.ptr<cv::Vec3b> (r);
    for (int c=0; c<converted.cols; c++){
      if (conv_r[c][2]>0){
	cur_ind = getIndex(conv_r[c][0], conv_r[c][1]);
	hist.at<double>(cur_ind.x, cur_ind.y)+=static_cast<double>(1.0);
      }
    }
  }
  
}

cv::Point ColorHistogram::getIndex(uchar h, uchar s){
  cv::Point hs;
  hs.x = static_cast<int>(h/_h_size);
  hs.y = static_cast<int>(s/_s_size);
  return hs;
}

void ColorHistogram::normalize()
{
  object_histogram /= static_cast<double>(sum(object_histogram).val[0]);
  background_histogram /= static_cast<double>(sum(background_histogram).val[0]);
  return;
}


void ColorHistogram::displayHists()
{
  normalize();
  
  cv::imshow("Object", object_histogram);
  cv::imshow("World", background_histogram);

  // cout << "***************Object***************" <<
  //   object_histogram << endl;

  // cout << "***************World***************" <<
  //   background_histogram << endl;

  cv::waitKey(0);
}

void ColorHistogram::writeToFile(string path)
{
  // //file names
  // string w_fn = "world.csv";
  // string o_fn = "object.csv";
  
  // //open files
  // ofstream object_f((path+o_fn).c_str());
  // ofstream world_f((path+w_fn).c_str());
  
  // // write the contents
  // object_f << format(object_histogram, "csv") << endl;
  // world_f << format(background_histogram, "csv") << endl;
  
  // //close
  // object_f.close();
  // world_f.close();

  cv::FileStorage fs(path+"robot_hist.yml", cv::FileStorage::WRITE);
  fs << "object" << object_histogram;
  fs << "world" << background_histogram;
}

void ColorHistogram::readFromFile(string path)
{
  //read file back
  cv::FileStorage fs(path+"robot_hist.yml", cv::FileStorage::READ);
  fs["robot"] >> object_histogram;
  fs["world"] >> background_histogram;
  
  cv::Mat temp;
  fs["world"] >> temp;
  
  cout << "World size " << background_histogram.size() << endl;
  cout << "Object size " << object_histogram.size() << endl;
  cout << "Temp size " << temp.size() << endl;
}

ColorHistogram::ColorHistogram(int bins, string path, int a_count/*=1*/)
{
  _thresh = THRESH;
  float h_range=181.0;
  float s_range=256.0;

  _bins_s = bins;
  _bins_h = 181;

  object_histogram = static_cast<double> (a_count) * 
    cv::Mat::ones(_bins_h, _bins_s, CV_64FC1);

  background_histogram = static_cast<double> (a_count) * 
    cv::Mat::ones(_bins_h, _bins_s, CV_64FC1);

  //debug
  cout << "World size " << background_histogram.size() << endl;
  cout << "Object size " << object_histogram.size() << endl;


  _h_size = static_cast<float>(h_range/_bins_h);
  _s_size = static_cast<float>(s_range/_bins_s);

  readFromFile(path);
  cout << "Done constructing.." << endl;
  return;
}

void ColorHistogram::testImg(const cv::Mat &im, cv::Mat &fore)
{
  //convert frame to HSV
  cv::Mat converted;
  cv::cvtColor(im, converted, CV_BGR2HSV);
  cv::Point cur_ind;

  fore = cv::Mat::zeros(im.size(), CV_8UC1);
  // fore = cv::Scalar(0);

  //debug
  cout << "Hist-back" << object_histogram.size() << endl;
  cout << "Hist-Robo" << background_histogram.size() << endl;
  
  for (int r=0; r<converted.rows; r++){
    cv::Vec3b* conv_r = converted.ptr<cv::Vec3b> (r);
    uchar* fore_r = fore.ptr<uchar> (r);
    for (int c=0; c<converted.cols; c++){
      if (conv_r[c][2]>0){
	cur_ind = getIndex(conv_r[c][0], conv_r[c][1]);
	double prob = object_histogram.at<double>(cur_ind.x, cur_ind.y) /
	  background_histogram.at<double>(cur_ind.x, cur_ind.y);
	if (prob < _thresh)
	  fore_r[c] = (uchar)255;
      }
    }
  }
  
  //cout << "Displayed a frame.." << endl;
  //cv::waitKey(10);
}
