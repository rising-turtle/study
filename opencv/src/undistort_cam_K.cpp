/*
 *  compute the new camera intrinsic parameters after undistortion 
 *
 * */

#include <iostream> // for standard I/O
#include <string>   // for strings
#include <iomanip>  // for controlling float print precision
#include <sstream>  // string to number conversion
#include <fstream>

#include <opencv2/core/core.hpp>        // Basic OpenCV structures (cv::Mat, Scalar)
#include <opencv2/imgproc/imgproc.hpp>  // Gaussian Blur
#include <opencv2/calib3d/calib3d.hpp>
// #include <opencv2/videoio/videoio.hpp>
#include <opencv2/highgui/highgui.hpp>  // OpenCV window I/O

using namespace std;
using namespace cv;

int main()
{
  // Brown–Conrady model 
  cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_32F);
  distCoeffs.at<float>(0,0) = -0.077133; // k1 radial distortion coefficient
  distCoeffs.at<float>(1,0) = 0.053167; // k2
  distCoeffs.at<float>(2,0) = -0.000144; // p1 tangential distortion coefficient
  distCoeffs.at<float>(3,0) = 0.001798; // p2  

  // intrinsic camera model 
  cv::Mat K1 = cv::Mat::zeros(3, 3, CV_64F); 
  K1.at<double>(0,0) = 626.994202; // fx 
  K1.at<double>(1,1) = 633.193970; // fy 
  K1.at<double>(0,2) = 318.442291; // cx 
  K1.at<double>(1,2) = 241.116608; // cy 
  K1.at<double>(2,2) = 1; 
  
  cv::Mat K = cv::getOptimalNewCameraMatrix(K1, distCoeffs, cv::Size(640, 480), 0, cv::Size(640,480), 0, false); 

  cout <<"fx: "<<K.at<double>(0,0)<<endl
    <<"fy: "<<K.at<double>(1,1)<<endl
    <<"cx: "<<K.at<double>(0,2)<<endl
    <<"cy: "<<K.at<double>(1,2)<<endl;

  return 0; 
}
