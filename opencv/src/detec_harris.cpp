#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>

using namespace cv; 
using namespace std; 


void output_harris(cv::Mat& img); 

int main(int argc, char* argv[])
{
    if(argc < 2)
    {
	cout <<"usage: detect_harris [img]"<<endl;
	return -1; 
    }
    cv::Mat img = imread(argv[1], -1); 
    cv::Mat detect_img(img.rows/2, img.cols/2, CV_8UC1);
    cv::Mat harris(img.rows/2, img.cols/2, CV_32FC1); 
    
    cout <<"img size: "<<img.rows<<" * "<<img.cols<<endl; 

    cv::resize(img, detect_img, detect_img.size()); 
    cv::cornerHarris(detect_img, harris, 3, 3, 0.04); 
    cv::imshow("img", img); 
    cv::waitKey(10); 
    cv::imshow("detect_img", detect_img); 
    cv::waitKey(10); 
    cv::imshow("harris", harris); 
    output_harris(harris);
    cv::waitKey(0); 
    return 0; 
}

void output_harris(cv::Mat& img)
{
    ofstream ouf("harris_img.log"); 
    int cnt = 0; 
    for(int i=0; i<img.rows; i++)
    {
    for(int j=0; j<img.cols; j++)
    {	
	ouf<<img.at<float>(i,j)<<" ";
	if(((float*)(img.data+i*img.step1()))[j] > 1e-6)
	{
	    cnt++; 
	}
    }
	ouf<<endl; 
    }
    cout <<"total has "<<cnt<<" harris points!"<<endl; 
}

