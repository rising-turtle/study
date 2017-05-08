#include "warp.h"
#include <iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "node.h"



void PlaneProjector::setMatrix(const cv::Mat& M){

	CV_Assert(M.size() == cv::Size(3, 3) && M.type() == CV_64F);
	cv::Mat_<double> K_(M);
	H[0] = K_(0,0); H[1] = K_(0,1); H[2] = K_(0,2);
	H[3] = K_(1,0); H[4] = K_(1,1); H[5] = K_(1,2);
	H[6] = K_(2,0); H[7] = K_(2,1); H[8] = K_(2,2);
	cv::Mat_<double> Rinv = M.inv();
	rH[0] = Rinv(0,0); rH[1] = Rinv(0,1); rH[2] = Rinv(0,2);
	rH[3] = Rinv(1,0); rH[4] = Rinv(1,1); rH[5] = Rinv(1,2);
	rH[6] = Rinv(2,0); rH[7] = Rinv(2,1); rH[8] = Rinv(2,2);
}

void PlaneProjector::mapForward(float x, float y, float &u, float &v)
{
	float x_ = H[0] * x + H[1] * y + H[2];
	float y_ = H[3] * x + H[4] * y + H[5];
	float z_ = H[6] * x + H[7] * y + H[8];
	
	u = x_/z_;
	v = y_/z_;

	if(fabs(u)>10000 || fabs(v)>10000){
		u = u;
	}
}

void PlaneProjector::mapBackward(float u, float v, float &x, float &y)
{
	float x_ = rH[0] * u + rH[1] * v + rH[2];
	float y_ = rH[3] * u + rH[4] * v + rH[5];
	float z_ = rH[6] * u + rH[7] * v + rH[8];
	
	x = x_/z_;
	y = y_/z_;
}

CWarp::CWarp():m_interpolate_mode(cv::INTER_LANCZOS4),m_boarder_mode(cv::BORDER_CONSTANT){}
CWarp::~CWarp(){}

void CWarp::findBoarder(cv::Point& l_tl, cv::Point& l_br, cv::Point& g_tl, cv::Point& g_br)
{
	if(g_tl.x>l_tl.x) g_tl.x = l_tl.x;
	if(g_tl.y>l_tl.y) g_tl.y = l_tl.y;
	if(g_br.x<l_br.x) g_br.x = l_br.x;
	if(g_br.y<l_br.y) g_br.y = l_br.y;
}

void CWarp::warpPlane(std::vector<CNode*> nodes, cv::Mat& result)
{
	if(nodes.size()<=0) return ;
	cv::Point g_tl; // global top-left 
	cv::Point g_br;	// global bottom-right
	
	g_tl.x = 0; // std::numeric_limits<float>::max();
	g_tl.y = 0; //std::numeric_limits<float>::max();
	g_br.x = -std::numeric_limits<float>::max();
	g_br.y = -std::numeric_limits<float>::max();
	
	cv::Point l_tl;  // local top-left
	cv::Point l_br;  // local bottom-right

	float max_y ; 
	std::vector<cv::Mat> images_(nodes.size());		  // record each image
	std::vector<cv::Point> start_point(nodes.size()); // record start point for each image
	std::vector<cv::Mat> image_mask(nodes.size());
	std::vector<bool> valid_image(nodes.size(),false);

	// 1 Warp each image and record these warped images 
	cv::Mat warped_image; 
	for(int i=0;i<nodes.size();i++){
		if(!warpPerspective(nodes[i]->m_img,warped_image,nodes[i]->m_HMatrix,l_tl,l_br))
			continue;
		images_[i]=warped_image.clone();
		valid_image[i] = true;
		start_point[i] = l_tl;
	// 2 find the boarder of the final image
		findBoarder(l_tl,l_br,g_tl,g_br);
	}
	
	// 3 merge all the local images into the final image
	// result.create(g_br.y-g_tl.y+1, g_br.x-g_tl.x+1,images_[0].type());
	result = cv::Mat::zeros(g_br.y-g_tl.y+1, g_br.x-g_tl.x+1,images_[0].type());
	cv::Point trans_shift(g_tl);

	for(int i=0;i<images_.size();i++)
	{
		if(!valid_image[i]) continue;
		cv::Mat obj_image(result,cv::Rect(start_point[i].x-trans_shift.x,\
			start_point[i].y-trans_shift.y,images_[i].cols,images_[i].rows));
		// images_[i].copyTo(obj_image);
		cv::addWeighted(obj_image,0.5,images_[i],0.4,0,obj_image);
	}
	return ;
}

bool CWarp::warpPerspective(cv::Mat& src, cv::Mat& dst, cv::Mat& HMatrix,cv::Point& l_tl, cv::Point& l_br)
{
	if(!src.data){
		cerr<<"No data in image!"<<endl;
		throw 0;
	}
	cv::Mat xmap, ymap;
	cv::Rect dst_roi = buildMaps(src.size(), HMatrix, xmap, ymap);

	l_tl = dst_roi.tl();
	l_br = dst_roi.br();

	if(l_tl.x== l_br.x && l_tl.y == l_br.y){
		return false; 
	}

	dst.create(dst_roi.height + 1, dst_roi.width + 1, src.type());
	remap(src, dst, xmap, ymap, m_interpolate_mode, m_boarder_mode);

	return true;//dst_roi.tl();
}

cv::Rect CWarp::buildMaps(cv::Size& src_size,const cv::Mat& H, cv::Mat& xmap, cv::Mat& ymap)
{
	m_projector.setMatrix(H);

	cv::Point dst_tl, dst_br;
	detectResultRoi(src_size, dst_tl, dst_br);

	size_t width = dst_br.x - dst_tl.x + 1;
	size_t height =  dst_br.y - dst_tl.y + 1;
	if(fabs((float)width)>2000 || fabs((float)height)>2000){
		cout<<"err width: "<<width<<" or height: "<<height<<endl;
		return cv::Rect(cv::Point(0,0),cv::Point(0,0));
	}
	xmap.create(height,width , CV_32F);
	ymap.create(height, width, CV_32F);

	float x, y;
	for (int v = dst_tl.y; v <= dst_br.y; ++v)
	{
		for (int u = dst_tl.x; u <= dst_br.x; ++u)
		{
			m_projector.mapBackward(static_cast<float>(u), static_cast<float>(v), x, y);
			xmap.at<float>(v - dst_tl.y, u - dst_tl.x) = x;
			ymap.at<float>(v - dst_tl.y, u - dst_tl.x) = y;
		}
	}

	return cv::Rect(dst_tl, dst_br);
}

void CWarp::detectResultRoi(cv::Size src_size, cv::Point &dst_tl, cv::Point &dst_br)
{
	float tl_uf = 0;// std::numeric_limits<float>::max();
	float tl_vf = 0;// std::numeric_limits<float>::max();
	float br_uf = -std::numeric_limits<float>::max();
	float br_vf = -std::numeric_limits<float>::max();

	float u, v;
	for (int y = 0; y < src_size.height; ++y)
	{
		for (int x = 0; x < src_size.width; ++x)
		{
			m_projector.mapForward(static_cast<float>(x), static_cast<float>(y), u, v);
			tl_uf = std::min(tl_uf, u); tl_vf = std::min(tl_vf, v);
			br_uf = std::max(br_uf, u); br_vf = std::max(br_vf, v);
		}
	}

	dst_tl.x = static_cast<int>(tl_uf);
	dst_tl.y = static_cast<int>(tl_vf);
	dst_br.x = static_cast<int>(br_uf);
	dst_br.y = static_cast<int>(br_vf);
}


void testWarp()
{
	// string filedir("D:\\myproj\\Stitching\\Stitching\\lenovo\\record.log");
	string filedir("D:\\myproj\\Stitching\\Stitching\\Lenovo_outdoor\\test3\\record.log");
	ifstream infile(filedir.c_str());
	char buf_line[4096];
	try
	{	
		vector<CNode*> nodes;
		CNode* cur_node;
		CNode* pre_node;
		cv::Mat rel_HMatrix;
		int cnt = -1;
		while(infile.getline(buf_line,4096)){
			++cnt;
			cur_node= new CNode(buf_line);
			if(nodes.size()<=0){
				nodes.push_back(cur_node);
				continue;
			}		
			pre_node = nodes[nodes.size()-1];
			if(!cur_node->matchNodePair(pre_node,rel_HMatrix)){
				cout<<"delete node: "<<cnt<<endl;
				continue;
			}
			nodes.push_back(cur_node);
		}
		if(nodes.size()<=1){
			cout<<"too few nodes!"<<endl;
			return ;
		}
		
		// warp those two nodes
		cv::Mat result;
		CWarp myWarp;
		myWarp.warpPlane(nodes,result);

		// Display the warp image
		cv::namedWindow("After warping");
		cv::imshow("After warping",result);

		cv::waitKey();
		IplImage* newImg = new IplImage(result);
		cout<<"new IplImage obtained!"<<endl;
		cvSaveImage("result.jpg",newImg);
		cout<<"finish saving!"<<endl;
		//cvReleaseImage(&newImg);
	}
	catch (...)
	{
		cout<<"failed to read images!"<<endl;
	}
	return ;
}