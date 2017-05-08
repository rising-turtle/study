#ifndef SYNCHRONI_GROUND_TRUTH
#define SYNCHRONI_GROUND_TRUTH

#include "preheader.h"
#include "opencv2/core/core.hpp"  
#include "opencv2/highgui/highgui.hpp" 

#include "CPose3D.h"

class CSynchroniGroundTruth{
public:
	CSynchroniGroundTruth();
	~CSynchroniGroundTruth();
public:
	vector<string> m_depth_name;
	vector<string> m_rgb_name;
	vector<CPose3D*> m_ground_truth;
	
	// ��������,��ʼ������Ǽ����޷�ͬ����֡
	int m_MWindow;

public:

	// ��ʼ����ȡ�ļ�����
	void init();

	// ��ȡgroundtruth����
	void readGroundTruth(string file_name,vector<double>& timest);


	// ��ȡ�ļ�����
	void readfile(string file_name,vector<double>& times, vector<string>& file_n);


	// �ҵ���ӽ���ֵ
	int findExactIndex(int from_, vector<double>& v_set, double key);


	// �ҵ���Ӧ���±�
	void SynIndex(vector<double> depth_t, vector<double> rgb_t,vector<double> gt_t,\
		vector<vector<int> >& match_index);


	// ͬ���ҵ���ƥ���֡
	void findSynchroniMatch(string depth_f, string rgb_f, string gt_f);

	// �����Щƥ�����Ϣ
	void dumpMatch(ostream& out, vector<double>& depth_t, vector<double>& rgb_t,\
		vector<double>& gt_t,vector<vector<int> >& match_index);

	// Ϊ�˺�ϵͳͬ���ӿ�
	int Run(unsigned char *pucRGB,unsigned short *pusD, \
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &m_PC);

	void calPCfromImageAndDepth(cv::Mat& image, cv::Mat& depth, \
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outPC);

	// ���ƥ���groundtruth
	void dumpMatchGT(ostream& out,vector<double>& gt_t, vector<vector<int> >& match_index);

	// �ҵ����е�ƥ�������Ϣ
	void findAllSynMatch(string depth_f, string rgb_f, string gt_f);

};

#endif