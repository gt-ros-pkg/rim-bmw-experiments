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

  float h_range=181.0;
  float s_range=256.0;

  object_histogram = static_cast<double> (a_count) * 
    cv::Mat::ones(bins, bins, CV_64FC1);

  background_histogram = static_cast<double> (a_count) * 
    cv::Mat::ones(bins, bins, CV_64FC1);

  _bins = bins;
  _h_size = static_cast<float>(h_range/_bins);
  _s_size = static_cast<float>(s_range/_bins);
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
  //file names
  string w_fn = "world.csv";
  string o_fn = "object.csv";
  
  //open files
  ofstream object_f((path+o_fn).c_str());
  ofstream world_f((path+w_fn).c_str());
  
  // write the contents
  object_f << format(object_histogram, "csv") << endl;
  world_f << format(background_histogram, "csv") << endl;
  
  //close
  object_f.close();
  world_f.close();
}

void ColorHistogram::readFromFile(string path)
{
  //file names
  string w_fn = "world.txt";
  string o_fn = "object.txt";

  //read from files
  CvMLData o_dat, w_dat;
  o_dat.read_csv((path+o_fn).c_str());
  w_dat.read_csv((path+w_fn).c_str());
  
  //write to the Mats
  const CvMat* o_tmp = o_dat.get_values();
  const CvMat* w_tmp = w_dat.get_values();
  
  cv::Mat o_mat(o_tmp, true);
  cv::Mat w_mat(w_tmp, true);

  o_mat.copyTo(object_histogram);
  w_mat.copyTo(background_histogram);
}

ColorHistogram::ColorHistogram(int bins, string path, int a_count/*=1*/)
{
  _thresh = THRESH;
  float h_range=181.0;
  float s_range=256.0;

  object_histogram = static_cast<double> (a_count) * 
    cv::Mat::ones(bins, bins, CV_64FC1);

  background_histogram = static_cast<double> (a_count) * 
    cv::Mat::ones(bins, bins, CV_64FC1);

  _bins = bins;
  _h_size = static_cast<float>(h_range/_bins);
  _s_size = static_cast<float>(s_range/_bins);

  readFromFile(path);
  return;
}

void ColorHistogram::testImg(const cv::Mat &im, cv::Mat &fore)
{
  //convert frame to HSV
  cv::Mat converted;
  cv::cvtColor(im, converted, CV_BGR2HSV);
  cv::Point cur_ind;

  fore = cv::Scalar(0);

  for (int r=0; r<converted.rows; r++){
    cv::Vec3b* conv_r = converted.ptr<cv::Vec3b> (r);
    uchar* fore_r = fore.ptr<uchar> (r);
    for (int c=0; c<converted.cols; c++){
      if (conv_r[c][2]>0){
	cur_ind = getIndex(conv_r[c][0], conv_r[c][1]);
	double prob = object_histogram.at<double>(cur_ind.x, cur_ind.y) /
	  background_histogram.at<double>(cur_ind.x, cur_ind.y);
	if (prob > _thresh)
	  fore_r[c] = (uchar)255;
      }
    }
  }
  
  //debug
  cv::Mat visuals;
  im.copyTo(visuals, fore);
  cv::imshow("The Filtered", visuals);
  cv::waitKey(10);
}
