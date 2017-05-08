#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "matcher.h"
#include "node.h"

int mytest(){
	try
	{
		CNode node1("..//lenovo//1.jpg");

	/*	IplImage * img = new IplImage(node1.m_img);
		cvSaveImage("result.jpg",img);
		return 0;*/

		CNode node2("..//lenovo//2.jpg");
		CNode node3("..//lenovo//3.jpg");
		
	

		cv::Mat HMatrix12;
		cv::Mat HMatrix23;
		node2.matchNodePair(&node1,HMatrix12);
		node3.matchNodePair(&node2,HMatrix23);
		cv::Mat HMatrix13 = HMatrix12*HMatrix23;

		// warp those two nodes
		cv::Mat result;
		cv::warpPerspective(node2.m_img, // input image
			result,			// output image
			HMatrix12,		// homography
			cv::Size(2*node2.m_img.cols,node2.m_img.rows)  // size of output image
			);
		cv::warpPerspective(node3.m_img,
			result,
			HMatrix13,
			cv::Size(2*node3.m_img.cols,node3.m_img.rows));
		// Copy image 1 on the first half of full image
		cv::Mat half(result,cv::Rect(0,0,node1.m_img.cols,node1.m_img.rows));
		node1.m_img.copyTo(half);

		// Display the warp image
		cv::namedWindow("After warping");
		cv::imshow("After warping",result);

		cv::waitKey();
	}
	catch (...)
	{
		cout<<"failed to read images!"<<endl;
	}
	return 0;
}