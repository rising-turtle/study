#ifndef _NODE_H_
#define _NODE_H_

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

class RobustMatcher;

class CNode{
public:
	CNode(string file);
	~CNode();
public:
	bool matchNodePair(CNode* prev,cv::Mat& homography);
	bool IsValidTrans(cv::Mat& M);
public:
	cv::Mat m_img; 
	RobustMatcher* m_matcher;
	vector<cv::KeyPoint> m_keyPts; // Key Points in this image
	cv::Mat m_keyDes;	// descriptors for these key points
	cv::Mat m_HMatrix;	// homography matrix relative to the root node
private:
	CNode(const CNode&);
	CNode& operator=(const CNode&);
};


#endif