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

void undist_tum_vio(int argc, char* argv[]); 

void test_getNewCamMatrix(); 
template<typename T>
void printMat(cv::Mat& m, string prefix="")
{
    cout<<prefix<<endl;
    for(int r=0; r<m.rows; r++)
    {
	for(int c=0; c<m.cols; c++)
	{
	    cout<<m.at<T>(r,c)<<" ";
	}
	cout<<endl;
    }
}

int main(int argc, char* argv[])
{
    undist_tum_vio(argc, argv); 
    // test_getNewCamMatrix(); 
    return 0; 
}


void undist_tum_vio(int argc, char* argv[])
{
    if(argc < 2)
    {
	cout<<"usage: ./* + *.[png]"<<endl; 
	return; 
    }
    cv::Mat img0 = cv::imread(argv[1], -1); 
    cv::Mat img1; 
    cv::Mat map1, map2; 
    cv::Mat K = cv::Mat::eye(3,3,CV_32F); 
    K.at<float>(0,0) = 190.9785; // fx;
    K.at<float>(1,1) = 190.9733; // fy;
    K.at<float>(0,2) = 254.9317; // cx;
    K.at<float>(1,2) = 256.8974; // cy;

    cv::Mat DistCoef(5,1,CV_32F);
    DistCoef.at<float>(0) = -0.239552 ; // -0.2847798; // fSettings["Camera.k1"];
    DistCoef.at<float>(1) = 0.037056;  // 0.08245052; // fSettings["Camera.k2"];
    DistCoef.at<float>(2) = 3.5763956e-6; // -1.0946156e-6 ; // fSettings["Camera.p1"];
    DistCoef.at<float>(3) = -1.4032145e-5; // 4.78701072e-6;  // fSettings["Camera.p2"];
    DistCoef.at<float>(4) = 0.;// -0.0104085; //k3;
    
    // cv::Mat newK = cv::Mat::eye(3,3,CV_32F); 
    cv::Mat newK; 
    cv::Size image_size(512, 512);
    printMat<float>(K, "oldK");
    cv::initUndistortRectifyMap(K, DistCoef,
                              cv::Mat(), newK, image_size,
                              CV_8UC1, map1, map2);
    cv::remap(img0, img1, map1, map2, cv::INTER_LINEAR,
	    cv::BORDER_CONSTANT, cv::Scalar());
    printMat<float>(newK, "newK"); 
    cv::Mat optK = cv::getOptimalNewCameraMatrix(K, DistCoef, image_size, 0, image_size, 0, false); 
    printMat<float>(optK, "optK"); 

    cv::imshow("undistorted img", img1); 
    cv::waitKey(0); 
    
    return ; 
}


void test_getNewCamMatrix()
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
}

