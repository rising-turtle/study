#ifndef SYNCHRONI_GROUND_TRUTH
#define SYNCHRONI_GROUND_TRUTH

#include "preheader.h"
#include "CPose3D.h"

class CSynchroniGroundTruth{
public:
	CSynchroniGroundTruth();
	~CSynchroniGroundTruth();
public:
	vector<string> m_depth_name;
	vector<string> m_rgb_name;
	vector<CPose3D*> m_ground_truth;
	
	// 用来过滤,开始和最后那几个无法同步的帧
	int m_MWindow;

public:

	// 初始化读取文件内容
	void init();

	// 读取groundtruth内容
	void readGroundTruth(string file_name,vector<double>& timest);


	// 读取文件内容
	void readfile(string file_name,vector<double>& times, vector<string>& file_n);


	// 找到最接近的值
	int findExactIndex(int from_, vector<double>& v_set, double key);


	// 找到对应的下标
	void SynIndex(vector<double> depth_t, vector<double> rgb_t,vector<double> gt_t,\
		vector<vector<int> >& match_index);


	// 同步找到最匹配的帧
	void findSynchroniMatch(string depth_f, string rgb_f, string gt_f);

	// 输出这些匹配的信息
	void dumpMatch(ostream& out, vector<double>& depth_t, vector<double>& rgb_t,\
		vector<double>& gt_t,vector<vector<int> >& match_index);

	// 输出匹配的groundtruth
	void dumpMatchGT(ostream& out,vector<double>& gt_t, vector<vector<int> >& match_index);

	// 找到所有的匹配深度信息
	void findAllSynMatch(string depth_f, string rgb_f, string gt_f);

	// 为了和系统同步接口
	/*int Run(unsigned char *pucRGB,unsigned short *pusD, \
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &m_PC);

	void calPCfromImageAndDepth(cv::Mat& image, cv::Mat& depth, \
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outPC);*/

};

#endif