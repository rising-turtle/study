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

void undist_fpv_vio(int argc, char* argv[]); 

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
    undist_fpv_vio(argc, argv); 
    // undist_tum_vio(argc, argv); 
    // test_getNewCamMatrix(); 
    return 0; 
}

void undist_fpv_vio(int argc, char* argv[])
{
    if(argc < 2)
    {
	cout<<"usage: ./* + *.[png]"<<endl; 
	return; 
    }
    cv::Mat img0 = cv::imread(argv[1], -1); 
    cv::Mat img1; 
    cv::Mat img2; 
    cv::Mat map1, map2; 
    cv::Mat K = cv::Mat::eye(3,3,CV_32F); 
    // FPV forward left cam K: 278.66723066149086, 278.48991409740296, 319.75221200593535, 241.96858910358173
    // FPV forward right cam K: 277.61640629770613, 277.63749695723294, 314.8944703346039, 236.04310050462587

    K.at<float>(0,0) = 278.66723066149086; // fx;
    K.at<float>(1,1) = 278.48991409740296; // fy;
    K.at<float>(0,2) = 319.75221200593535; // cx;
    K.at<float>(1,2) = 241.96858910358173; // cy;

    // FPV forward left cam D: -0.013721808247486035, 0.020727425669427896, -0.012786476702685545, 0.0025242267320687625
    // FPV forward right cam D: -0.008456929295619607, 0.011407590938612062, -0.006951788325762078, 0.0015368127092821786

    cv::Mat DistCoef(5,1,CV_32F);
//    DistCoef.at<float>(0) = -0.239552 ; // -0.2847798; // fSettings["Camera.k1"];
//    DistCoef.at<float>(1) = 0.037056;  // 0.08245052; // fSettings["Camera.k2"];
//    DistCoef.at<float>(2) = 3.5763956e-6; // -1.0946156e-6 ; // fSettings["Camera.p1"];
//    DistCoef.at<float>(3) = -1.4032145e-5; // 4.78701072e-6;  // fSettings["Camera.p2"];
//    DistCoef.at<float>(4) = 0.;// -0.0104085; //k3;
    
   //  float dist_data[5] = {0.320760, -0.864822, 0, 0, 0.589437}; 
    float dist_data[5] = {-0.013721808247486035, 0.020727425669427896, -0.012786476702685545, 0.0025242267320687625, 0} ;// 0.320760, -0.864822, 0, 0, 0.589437}; 

    cv::Mat distCoeffs(1, 5, CV_32F, dist_data);  
    float cam_matrix_data[9] = {444.277, 0, 324.055, 0, 444.764, 254.516, 0, 0, 1};  
    cv::Mat cameraMatrix(3, 3, CV_32F, cam_matrix_data); 
    
    // K = cameraMatrix; 
    DistCoef = distCoeffs; 


    // cv::Mat newK = cv::Mat::eye(3,3,CV_32F); 
    cv::Mat newK; 
    // cv::Size image_size(512, 512);
    // cv::Size image_size(480, 320); // cv::Size(width, height)
    cv::Size image_size(640, 480); // cv::Size(width, height)

    printMat<float>(K, "oldK");

    float equidistant_distort[4] = {-0.013721808247486035, 0.020727425669427896, -0.012786476702685545, 0.0025242267320687625}; 
    cv::Mat equidistant_distCoeffs(1,4, CV_32F, equidistant_distort); 

    // adjust camera matrix according to the scale parameters alpha \in [0, 1] 
    cv::Mat optK = cv::getOptimalNewCameraMatrix(K, DistCoef, image_size, 0., image_size, 0, false); 
    // cv::Mat optK = cv::getOptimalNewCameraMatrix(K, DistCoef, image_size, 1); 

    // fisheye 
    cv::Mat R = cv::Mat::eye(3,3,CV_32F); 
    cv::Mat P = cv::Mat::eye(3,3,CV_32F); 
    
    // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, equidistant_distCoeffs, image_size, R, P, 0, image_size);
    cv::Size new_img_size(640, 480); 
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, equidistant_distCoeffs, image_size, R, P, 0, new_img_size);
    printMat<float>(P, "fisheye_newP"); 
	
    // cout<<"new image size: width: "<<new_img_size.width<<" height:"<<new_img_size.height<<endl;


    // remap, now it works,  
    cv::Mat M1l,M2l,M1r,M2r;
    cv::fisheye::initUndistortRectifyMap(K,equidistant_distCoeffs,R,P,new_img_size,CV_32F,M1l,M2l);
    cv::Mat imRect; 
    cv::remap(img0,imRect,M1l,M2l,cv::INTER_LINEAR);
 
    cv::Mat imRect2; 
    cv::fisheye::undistortImage(img0, imRect2, K, equidistant_distCoeffs); 

    cv::imshow("undistorted fisheye rect img", imRect); 
    cv::moveWindow("undistorted fisheye rect img", 40, 540); 
    
    cv::imshow("undistorted fisheye rect img2", imRect2); 
    cv::moveWindow("undistorted fisheye rect img2", 540, 540); 

    cout <<"imRect1.size: "<<imRect.cols<<" "<<imRect.rows<<endl; 
    cout <<"imRect2.size: "<<imRect2.cols<<" "<<imRect2.rows<<endl; 


   //  cv::initUndistortRectifyMap(K, DistCoef,
   //                           cv::Mat(), K, image_size,
   //                           CV_8UC1, map1, map2);
   // cv::remap(img0, img1, map1, map2, cv::INTER_LINEAR,
//	    cv::BORDER_CONSTANT, cv::Scalar());

    // printMat<float>(K, "newK"); 
    printMat<float>(optK, "optK"); 
    
    cv::undistort(img0, img1, K, DistCoef, optK);
    cv::undistort(img0, img2, K, DistCoef); // default means K, the new image is based on K only
    
    cout <<"img1.size: "<<img1.cols<<" "<<img1.rows<<endl; 
    cout <<"img2.size: "<<img2.cols<<" "<<img2.rows<<endl; 


    cv::imshow("undistorted img default", img2); 
    cv::moveWindow("undistorted img default", 40, 40); 
//    cv::waitKey(0); 

    cv::imshow("undistorted img using optK", img1); 
    cv::moveWindow("undistorted img using optK", 40 + 640 + 100, 40); 
    cv::waitKey(0); 
    
    return ; 
 

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
    cv::Mat img2; 
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
    
   //  float dist_data[5] = {0.320760, -0.864822, 0, 0, 0.589437}; 
    float dist_data[5] = {0} ;// 0.320760, -0.864822, 0, 0, 0.589437}; 

    cv::Mat distCoeffs(1, 5, CV_32F, dist_data);  
    float cam_matrix_data[9] = {444.277, 0, 324.055, 0, 444.764, 254.516, 0, 0, 1};  
    cv::Mat cameraMatrix(3, 3, CV_32F, cam_matrix_data); 
    
    K = cameraMatrix; 
    DistCoef = distCoeffs; 


    // cv::Mat newK = cv::Mat::eye(3,3,CV_32F); 
    cv::Mat newK; 
    // cv::Size image_size(512, 512);
    // cv::Size image_size(480, 320); // cv::Size(width, height)
    cv::Size image_size(640, 480); // cv::Size(width, height)

    printMat<float>(K, "oldK");

    // adjust camera matrix according to the scale parameters alpha \in [0, 1] 
   // cv::Mat optK = cv::getOptimalNewCameraMatrix(K, DistCoef, image_size, 0., image_size, 0, false); 
    cv::Mat optK = cv::getOptimalNewCameraMatrix(K, DistCoef, image_size, 0); 


   //  cv::initUndistortRectifyMap(K, DistCoef,
   //                           cv::Mat(), K, image_size,
   //                           CV_8UC1, map1, map2);
   // cv::remap(img0, img1, map1, map2, cv::INTER_LINEAR,
//	    cv::BORDER_CONSTANT, cv::Scalar());

    printMat<float>(K, "newK"); 
    printMat<float>(optK, "optK"); 
    
    cv::undistort(img0, img1, K, DistCoef, optK);
    cv::undistort(img0, img2, K, DistCoef); // default means K, the new image is based on K only
    
    cv::imshow("undistorted img default", img2); 
    cv::moveWindow("undistorted img default", 40, 40); 
//    cv::waitKey(0); 

    cv::imshow("undistorted img using optK", img1); 
    cv::moveWindow("undistorted img using optK", 40 + 640 + 100, 40); 
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

