
/**
	只是临时针对特定的tag进行抽取，并且我们的例子文件中，tag
	都居于图像的中心
**/
#ifndef TAG_FINDER_H
#define TAG_FINDER_H

#include "preheader.h"
#include "globaldefinition.h"
#include "opencv2/core/core.hpp"  
#include "opencv2/highgui/highgui.hpp" 

class CTagFinder
{
public:
	CTagFinder();
	~CTagFinder();

	cv::Mat m_image;
	int m_l,m_r,m_u,m_d;
	bool m_IsReady;
	
	// 读取图像文件
	void getImage(string file_name);
	// 计算tag所在的像素框
	void calTagLoc();
	// 显示所画tag区域
	void showTagLoc(int l,int r,int u,int d);
	// 中值滤波
	void MidFilter();

	// 利用拉普拉斯找边沿
	void FindEdge();
	void FindValidEdge(vector<vector<bool> >&);

	// 利用Sobel算子找边沿
	void SobelFindEdge();

	// 清除数据
	void clear();

protected:
private:
};

#endif