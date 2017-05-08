#ifndef _WARP_H_
#define _WARP_H_

#include <vector>
#include <opencv2/core/core.hpp>

class CNode;

struct PlaneProjector
{
public:
	void mapForward(float x, float y, float &u, float &v);
	void mapBackward(float u, float v, float &x, float &y);
	void setMatrix(const cv::Mat& M);
	double H[9];
	double rH[9];
};

class CWarp{
public:
	CWarp();
	~CWarp();
public:
	void warpPlane(std::vector<CNode*> nodes, cv::Mat& result);
	bool warpPerspective(cv::Mat& src, cv::Mat& dst, cv::Mat& HMatrix, cv::Point&, cv::Point&);
	cv::Rect buildMaps(cv::Size& ,const cv::Mat& HMatrix, cv::Mat& xmap, cv::Mat& ymap);
	void detectResultRoi(cv::Size src_size, cv::Point &dst_tl, cv::Point &dst_br);
	void findBoarder(cv::Point& l_tl, cv::Point& l_br, cv::Point& g_tl, cv::Point& g_br);
public:
	PlaneProjector m_projector;
	int m_interpolate_mode;
	int m_boarder_mode;
private:
	CWarp(const CWarp&);
	CWarp& operator=(const CWarp&);
};


#endif