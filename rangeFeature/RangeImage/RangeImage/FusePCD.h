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
	// �ϳ����������ĵ���ͼ
	bool compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_pc);
	bool compute_ORB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_pc);

	// �ӵ�������ȡ2D-RGB��Ϣ��Rangeͼ����Ϣ
	void getImagesandDepthMetaData(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud,
		unsigned char* rgbbuf,unsigned char* depthbuf);
	// ����Pose_Node
	Node* createFeatureNode(const cv::Mat& visual , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& point_cloud, 
		const cv::Mat& depth);
	// ����Pose_Node ORB
	COrbNode* createFeatureNodeORB(const cv::Mat& visual , \
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& point_cloud);

	// slam on ground truth data_set
	bool computeGT();
public:
	string m_file_dir;			// �ļ�·��
	GraphManager m_graph_mgr;	// ����Hogmanͼ�ṹ
	CLogfile mylogfile;			// ��¼����ģ������ʱ��

	CSynchroniGroundTruth m_gt;

	//OpenCV variables 
	cv::Ptr<cv::FeatureDetector> m_detector;
	cv::Ptr<cv::DescriptorExtractor> m_extractor;
	cv::Ptr<cv::DescriptorMatcher > m_matcher;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif