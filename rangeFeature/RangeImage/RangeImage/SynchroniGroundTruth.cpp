#include "preheader.h"
#include "SynchroniGroundTruth.h"
#include <iomanip>

void CSynchroniGroundTruth::calPCfromImageAndDepth(cv::Mat& image, cv::Mat& depth, \
											 pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outPC)
{
	outPC->header.frame_id = "/openni_rgb_optical_frame";// rgb_frame_id_;

	// just guess
	int depth_width_ = 640;
	int depth_height_ = 480;
	int image_width_ = 640;
	int image_height_ = 480;

	outPC->height = depth_height_;
	outPC->width = depth_width_;
	outPC->is_dense = false;

	outPC->points.resize(outPC->height * outPC->width);

	register int centerX = (outPC->width >> 1);
	int centerY = (outPC->height >> 1);
	// 这是 1/f of Kinect
	static const double constant=0.0019047619;

	// depth_image already has the desired dimensions, but rgb_msg may be higher res.
	register int color_idx = 0, depth_idx = 0;

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	register int wi=0,ci=0,cj=0;

	for (int v = -centerY; v < centerY; ++v,++wi)
	{
		for (register int u = -centerX; u < centerX; ++u,cj+=3,ci++,depth_idx++)
		{
			pcl::PointXYZRGB& pt = outPC->points[depth_idx];
			/// @todo Different values for these cases
			// Check for invalid measurements
			if (depth.at<unsigned short>(wi,ci) == 0)
				/*depth[depth_idx] == depth_image->getNoSampleValue() ||
				depth[depth_idx] == depth_image->getShadowValue())*/
			{
				pt.x = pt.y = pt.z = bad_point;
			}
			else
			{
				pt.z = depth.at<unsigned short>(wi,ci)* 0.001f;
				pt.x = u * pt.z * constant;
				pt.y = v * pt.z * constant;
			}

			// Fill in color
			pt.r = image.at<unsigned char>(wi,cj);
			pt.g = image.at<unsigned char>(wi,cj + 1);
			pt.b = image.at<unsigned char>(wi,cj + 2);
		}
		cj=0;
		ci=0;
	}
}


CSynchroniGroundTruth::CSynchroniGroundTruth():m_MWindow(5){ /*init();*/}
CSynchroniGroundTruth::~CSynchroniGroundTruth(){
	for(int i=0;i<m_ground_truth.size();i++){
		delete m_ground_truth[i];
		m_ground_truth[i]=0;
	}
	m_ground_truth.clear();
}


void CSynchroniGroundTruth::init()
{
	//findSynchroniMatch("depth.txt","rgb.txt","groundtruth.txt");
	findAllSynMatch("depth.txt","rgb.txt","groundtruth.txt");
}

// 读取文件内容
void CSynchroniGroundTruth::readfile(string file_name, vector<double>& times, vector<string>& file_n)
{
	ifstream inputf(file_name.c_str());
	if(!inputf.is_open()){
		cout<<"failed to read file: "<<file_name<<endl;
		inputf.close();
		return ;
	}
	char buf[100];
	// 过滤掉无效行
	inputf.getline(buf,100);
	inputf.getline(buf,100);
	inputf.getline(buf,100);
	
	double timestamp;
	while(inputf.getline(buf,100))
	{
		char tmpbuf[100]={0};
		sscanf(buf,"%lf %s",&timestamp,tmpbuf);
		times.push_back(timestamp);
		file_n.push_back(string(tmpbuf));
	}
	inputf.close();
	assert(times.size()==file_n.size());
	return ;
}

// 读取groundtruth内容
void CSynchroniGroundTruth::readGroundTruth(string file_name,vector<double>& timest)
{
	ifstream inputf(file_name.c_str());
	
	if(!inputf.is_open())
	{
		cout<<"readGT: "<<file_name<< "failed!"<<endl;
		inputf.close();
		return;
	}

	char buf[100];
	// 过滤垃圾信息
	inputf.getline(buf,100);
	inputf.getline(buf,100);
	inputf.getline(buf,100);

	while(inputf.getline(buf,100))
	{
		float q[7];
		double time_stamp;
		sscanf(buf,"%lf %f %f %f %f %f %f %f", &time_stamp,\
			&q[0],&q[1],&q[2],&q[3],&q[4],&q[5],&q[6]);
		m_ground_truth.push_back(new CPose3D(q));
		timest.push_back(time_stamp);
	}
	inputf.close();
}

// 找到最接近的值
int CSynchroniGroundTruth::findExactIndex(int from_, vector<double>& v_set, double key)
{
	int from = from_;
	if(from<0 || from>=v_set.size())
	{
		cout<<"range error in findExactIndex()"<<endl;
		return -1;
	}
	double dis = fabs(v_set[from]-key);
	while(++from<v_set.size())
	{
		double n_dis = fabs(v_set[from]-key);
		if(n_dis<dis)
			dis = n_dis;
		else if(n_dis>dis)
			break;
	}
	return --from;
}

// 找到对应的下标
void CSynchroniGroundTruth::SynIndex(vector<double> depth_t, vector<double> rgb_t,vector<double> gt_t,\
			  vector<vector<int> >& match_index)
{
	// 我们以 depth_t 的值为依据来查找匹配的索引值
	
	if(depth_t.size() < 2*m_MWindow )
	{
		cout<<"depth size is too small"<<endl;
		return ;
	}

	// 去掉最前面和最后面M帧
	int first = m_MWindow;
	int last = depth_t.size()-m_MWindow;
	vector<int> tmpindex(3);
	
	int rgb_index = 0;
	int gt_index = 0;

	for(int i=first;i<=last;i++)
	{
		rgb_index = findExactIndex(rgb_index,rgb_t,depth_t[i]);
		gt_index = findExactIndex(gt_index,gt_t,depth_t[i]);

		// 1:depth 2:rgb 3:groundtruth
		tmpindex[0] = i;
		tmpindex[1] = rgb_index;
		tmpindex[2] = gt_index;

		match_index.push_back(tmpindex);
		rgb_index -=3; //为了防止漏掉最匹配的帧
		gt_index -=3;
	}
}

// 输出这些匹配的信息
void CSynchroniGroundTruth::dumpMatch(ostream& out, vector<double>& depth_t, vector<double>& rgb_t,\
			   vector<double>& gt_t,vector<vector<int> >& match_index)
{
	out<<"N"<<"\tdepth"<<"\trgb"<<"\tgt"<<endl;
	for(int i=0;i<match_index.size();i++){
		out<<i<<"\t"<<std::fixed<<depth_t[match_index[i][0]]<<"\t"<<std::fixed<<rgb_t[match_index[i][1]]<<"\t" \
			<<std::fixed<<gt_t[match_index[i][2]]<<endl;
		/*stringstream s1,s2,s3;
		s1<<std::fixed<<depth_t[match_index[i][0]];
		s2<<std::fixed<<rgb_t[match_index[i][1]];
		s3<<std::fixed<<gt_t[match_index[i][2]];
		out<<i+1<<"\t"<<s1.str()<<"\t"<<s2.str()<<"\t"<<s3.str()<<endl;*/
	}
}


// 同步找到最匹配的帧
void CSynchroniGroundTruth::findSynchroniMatch(string depth_f, string rgb_f, string gt_f)
{
	vector<double> depth_t,rgb_t,gt_t;
	vector<string> depth_n,rgb_n;
	readfile(depth_f,depth_t,depth_n);
	readfile(rgb_f,rgb_t,rgb_n);
	readGroundTruth(gt_f,gt_t);
	
	vector<vector<int> > match_index;
	SynIndex(depth_t,rgb_t,gt_t,match_index);
	
	ofstream output_gt("matched_ground_truth.txt");

	for(int i=0;i<match_index.size();i++)
	{
		m_depth_name.push_back(depth_n[match_index[i][0]]);
		m_rgb_name.push_back(rgb_n[match_index[i][1]]);
		CPose3D* pNode = m_ground_truth[match_index[i][2]];
		output_gt<<std::fixed<<gt_t[match_index[i][2]]<<" "<<pNode->m_coords[0] \
			<<" "<<pNode->m_coords[1]<<" "<<pNode->m_coords[2]<<" "<<pNode->q[0] \
			<<" "<<pNode->q[1]<<" "<<pNode->q[2]<<" "<<pNode->q[3]<<endl;
	}
			
	ofstream outfile("d:\\SynchroGroundTruth.txt");
	dumpMatch(outfile,depth_t,rgb_t,gt_t,match_index);
}


// 为了和系统接口同步
int CSynchroniGroundTruth::Run(unsigned char *pucRGB,unsigned short *pusD,boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > &m_PC)
{
	static int nCount=0;
	if(nCount==0)
		cout<<"Total Size: "<<m_depth_name.size()<<endl;
	if (nCount<m_depth_name.size())
	{
		cv::Mat image;
		cv::Mat depth;
		image=cv::imread(m_rgb_name[nCount]);
		depth=cv::imread(m_depth_name[nCount],2);

		memcpy(pucRGB,image.data,640*480*3);
		memcpy(pusD,depth.data,640*480*2);

		calPCfromImageAndDepth(image,depth,m_PC);
		nCount++;
		return 0;
	}
	else{ 
		cout<<"All images have been handled!"<<endl;
		return -1;
	}
}

// 找到所有的匹配深度信息
void CSynchroniGroundTruth::findAllSynMatch(string depth_f, string rgb_f, string gt_f)
{
	vector<double> depth_t,rgb_t,gt_t;
	vector<string> depth_n,rgb_n;
	readfile(depth_f,depth_t,depth_n);
	readfile(rgb_f,rgb_t,rgb_n);
	readGroundTruth(gt_f,gt_t);

	vector<vector<int> > match_index;
	vector<int> tmp_match(3);
	int gt_index = 0;
	for(int i=0;i<depth_t.size();i++)
	{
		gt_index = findExactIndex(gt_index,gt_t,depth_t[i]);
		tmp_match[0] = i; tmp_match[1] = i; tmp_match[2] = gt_index;
		match_index.push_back(tmp_match);
		//if(gt_index>2) gt_index--;
	}
	
	for(int i=0;i<match_index.size();i++)
	{
		m_depth_name.push_back(depth_n[match_index[i][0]]);
		m_rgb_name.push_back(rgb_n[match_index[i][1]]);
	}

	ofstream match_index_f("AllMatchSyn.txt");
	dumpMatchGT(match_index_f,gt_t,match_index);

	ofstream outfile("AllSynchroGroundTruth.txt");
	dumpMatch(outfile,depth_t,rgb_t,gt_t,match_index);
}

// 输出匹配的groundtruth
void CSynchroniGroundTruth::dumpMatchGT(ostream& out,vector<double>& gt_t, vector<vector<int> >& match_index)
{
	for(int i=0;i<match_index.size();i++)
	{
		CPose3D* pPose = m_ground_truth[match_index[i][2]];
		out<<std::fixed<<gt_t[match_index[i][2]]<<" "<<pPose->m_coords[0]<<" "\
			<<pPose->m_coords[1]<<" "<<pPose->m_coords[2]<<" "<<pPose->q[0]<<" "\
			<<pPose->q[1]<<" "<<pPose->q[2]<<" "<<pPose->q[3]<<endl;
	}
}