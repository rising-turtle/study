/*  
 *  Apr. 26 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  test stereo match given two images 
 *
 * */

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "image.h"
#include "elas.h"

using namespace std; 
using namespace cv; 

image<uchar>* fromMat(cv::Mat); 
cv::Mat fromImage(image<uchar>*);
void process(image<uchar>* I1, image<uchar>* I2); 

int main(int argc, char* argv[])
{
  string s1,s2; 
  if(argc >=3) 
  {
    s1 = argv[1]; 
    s2 = argv[2]; 
  }

  cv::Mat imL = imread(s1, CV_LOAD_IMAGE_UNCHANGED); 
  cv::Mat imR = imread(s2, CV_LOAD_IMAGE_UNCHANGED); 

  image<uchar>* I1 = fromMat(imL); 
  image<uchar>* I2 = fromMat(imR); 
  process(I1, I2); 
    
  return 0; 
}

void process(image<uchar>* I1, image<uchar>* I2)
{
  int32_t width  = I1->width();
  int32_t height = I1->height();
  const int32_t dims[3] = {width,height,width}; // bytes per line = width
  float* D1_data = (float*)malloc(width*height*sizeof(float));
  float* D2_data = (float*)malloc(width*height*sizeof(float));

  // prosess 
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(I1->data,I2->data,D1_data,D2_data,dims);

  // find maximum disparity 
  float disp_max = 0;
  for (int32_t i=0; i<width*height; i++) {
    if (D1_data[i]>disp_max) disp_max = D1_data[i];
    if (D2_data[i]>disp_max) disp_max = D2_data[i];
  }

  image<uchar> *D1 = new image<uchar>(width,height);
  image<uchar> *D2 = new image<uchar>(width,height);
  for (int32_t i=0; i<width*height; i++) {
    D1->data[i] = (uint8_t)max(255.0*D1_data[i]/disp_max,0.0);
    D2->data[i] = (uint8_t)max(255.0*D2_data[i]/disp_max,0.0);
  }

  cv::Mat imgD1 = fromImage(D1); 
  cv::Mat imgD2 = fromImage(D2); 
  cv::imshow("disparity D1", imgD1); 
  cv::imshow("disparity D2", imgD2); 
  cv::waitKey(0); 
  delete I1;
  delete I2;
  delete D1;
  delete D2;
  free(D1_data);
  free(D2_data);
}

image<uchar>* fromMat(Mat img)
{
  int width = img.cols; 
  int height = img.rows; 
  image<uchar> *im = new image<uchar>(width, height);
  
  memcpy((char*)imPtr(im, 0,0 ), (char*)img.data, width * height * sizeof(uchar)); 
  return im; 
}

cv::Mat fromImage(image<uchar>* I)
{
  int width = I->width(); 
  int height = I->height(); 
  cv::Mat ret(height, width, CV_8UC1); 
  memcpy((char*)ret.data, (char*)imPtr(I, 0,0), width * height * sizeof(uchar)); 
  return ret; 
}
