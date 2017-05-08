#ifndef FUSEPCD_H
#define FUSEPCD_H

#include "preheader.h"
#include "globaldefinition.h"
#include "graph_manager.h"
#include "CLogfile.h"
#include "SynchroniGroundTruth.h"

class Node;

/*
	This class will fuse *.pcd into a single point cloud,according to ascending number sequence  
*/

class CFusePcd{
public:
	CFusePcd(string file_dir);
	~CFusePcd();
public:
	// 合成整个场景的点云图
	bool compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_pc);
	bool compute_ORB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_pc);

	// 从点云中提取2D-RGB信息与Range图像信息
	void getImagesandDepthMetaData(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud,
		unsigned char* rgbbuf,unsigned char* depthbuf);
	// 生成Pose_Node
	Node* createFeatureNode(const cv::Mat& visual , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& point_cloud, 
		const cv::Mat& depth);
	// 生成Pose_Node ORB
	COrbNode* createFeatureNodeORB(const cv::Mat& visual , \
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& point_cloud);

	// slam on ground truth data_set
	bool computeGT();
public:
	string m_file_dir;			// 文件路径
	GraphManager m_graph_mgr;	// 管理Hogman图结构
	CLogfile mylogfile;			// 记录各个模块所用时间

	CSynchroniGroundTruth m_gt;

	//OpenCV variables 
	cv::Ptr<cv::FeatureDetector> m_detector;
	cv::Ptr<cv::DescriptorExtractor> m_extractor;
	cv::Ptr<cv::DescriptorMatcher > m_matcher;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif