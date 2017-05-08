#include "node.h"
#include <iostream>
#include <vector>
#include "matcher.h"

CNode::CNode(string file):m_matcher(new RobustMatcher){
	// 读取图像文件
	m_img = cv::imread(file.c_str(),0);
	if(!m_img.data){
		cerr<<"false to read img!"<<endl;
		throw 1;
	}
	// 设置特征参数
	m_matcher->setConfidenceLevel(0.98);
	m_matcher->setMinDistanceToEpipolar(1.0);
	m_matcher->setRatio(0.65f);	
	//cv::Ptr<cv::FeatureDetector> pfd=  cv::Algorithm::create<cv::FeatureDetector>("Feature2D.SURF"); //new cv::SurfFeatureDetector(10); 
	//m_matcher->setFeatureDetector(pfd);
	// 计算该图像的特征点
	m_matcher->getKeyPoints(m_img,m_keyPts,m_keyDes);
	
	// 初始化旋转矩阵
	m_HMatrix= cv::Mat::eye(3,3,CV_64FC1);
}

CNode::~CNode(){
	if(m_matcher!=NULL)
		delete m_matcher;
}

static const int minimal_trans_x = 10;
static const int minimal_trans_y = 5;
static const int minimal_matches = 10;
bool CNode::IsValidTrans(cv::Mat& M){
	CV_Assert(M.size() == cv::Size(3, 3) && M.type() == CV_64F);
	if(cv::saturate_cast<int>(M.at<double>(0,2))< minimal_trans_x && \
		cv::saturate_cast<int>(M.at<double>(1,2))<minimal_trans_y)
	{
		return false;
	}
	return true;
}

bool CNode::matchNodePair(CNode* prev,cv::Mat& homography){
	// Match the two images
	int n_matches= m_matcher->calHomographyMatrix(prev->m_img,m_img,prev->m_keyPts,\
		m_keyPts,prev->m_keyDes,m_keyDes,homography);
	if(!IsValidTrans(homography) || n_matches<minimal_matches){
		return false;
	}
	m_HMatrix = prev->m_HMatrix*homography;
	return true;
}