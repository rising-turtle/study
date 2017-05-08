#include <windows.h>
#include <tchar.h>
//#include <direct.h>
#include "boost/shared_array.hpp"
#include "FusePCD.h"
#include "ORBNode.h"

CFusePcd::CFusePcd(string file_dir):m_file_dir(file_dir)
{
	int npos=m_file_dir.rfind("\\");
	if(npos!=m_file_dir.size()-1) // 路径必须以"\\"结尾
	{
		m_file_dir.append("\\");
	}
	// 利用ORB特征匹配
	/*m_detector = new cv::DynamicAdaptedFeatureDetector(new cv::FastAdjuster(),
		global_adjuster_min_keypoints,
		global_adjuster_max_keypoints,
		global_surf_adjuster_max_iterations);
	m_extractor = cv::DescriptorExtractor::create("ORB");
	m_matcher = cv::DescriptorMatcher::create("BruteForce");*/

	// 初始化OpenCV特征提取的参数，SURF,BruteForce
	m_detector = new cv::DynamicAdaptedFeatureDetector(new cv::SurfAdjuster(),
		global_adjuster_min_keypoints,
		global_adjuster_max_keypoints,
		global_surf_adjuster_max_iterations);
	m_extractor = cv::DescriptorExtractor::create("SURF");
	m_matcher = cv::DescriptorMatcher::create("BruteForce");

	// 从GroundTruth中读取数据进行SLAM
	m_gt.init();
}

CFusePcd::~CFusePcd(){}


// slam on ground truth data_set
bool CFusePcd::computeGT()
{
	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	unsigned char* rgb_buf = new unsigned char[640*480*3];
	unsigned short* depth_buf = new unsigned short[640*480];
	double start_t_slam,start_t_fe;
	int n_nodes=0;
	while(1){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
		if(_LOG){ 
			// initialize log parameters
			gl_pf_gs=0; // graph_size
			gl_pf_fe=0;	// time for Feature Extraction
			gl_pf_fm=0; // time for Feature Match
			gl_pf_me=0; // time for Motion Estimation
			gl_pf_op=0; // time for Graph Optimization
			gl_pf_slam=0; // time for total Slam
		}
		if(m_gt.Run(rgb_buf,depth_buf,m_tmpPC)!=-1)
		{
			memcpy(cvRGBImage.data,rgb_buf,640*480*3);
			memcpy(cvDepthImage.data,(unsigned char*)depth_buf,640*480*2);
			cvDepthImage.convertTo(cvDepth8UC1Image,CV_8UC1);

			// record start time of SLAM
			start_t_slam = ::GetTickCount();

			if(_LOG) start_t_fe=::GetTickCount(); // 记录特征提取所需要的时间
			// 创建新的Pose节点
			Node* node_ptr = createFeatureNode(cvRGBImage, m_tmpPC, cvDepth8UC1Image);
			if(_LOG) gl_pf_fe=::GetTickCount() - start_t_fe;
			
			n_nodes++;
			// 加入到Hogman结构中
			if(!m_graph_mgr.addNode(node_ptr))
			{
				cout<<"false to add node: "<<n_nodes<<endl;
				delete node_ptr;
				continue;
				//return false;
			}
			else{
				cout<<"succeed to add node: "<<n_nodes<<endl;
				// 如果需要Log的话
				if(_LOG){
					gl_pf_slam=::GetTickCount()-start_t_slam;
					mylogfile.writeid(n_nodes-1); // id of frame
					double graphsize = (double)m_graph_mgr.graph_.size();
					mylogfile.writeintolog(graphsize); // graph size
					mylogfile.writeintolog(gl_pf_fe); // FE
					mylogfile.writeintolog(gl_pf_fm); // FM
					mylogfile.writeintolog(gl_pf_me); // R_ME
					mylogfile.writeintolog(gl_pf_op); // OP
					mylogfile.writeintolog(gl_pf_slam,true); // Total Slam
				}
			}
		}
		else
		{
			break;
		}
	}
	ofstream trajectory("d:\\MyProjects\\matlab\\trajectory.txt");
	m_graph_mgr.outputTrajectory(trajectory);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	m_graph_mgr.FuseOutPC(out_pc);
	DisplayPT(out_pc);
	return true;
}

// ASCII version
bool CFusePcd::compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_pc)
{
	static int num_pcds=10; // 文件个数
	static int pcd_fst=1;	// 起始文件
	string file_name;		// 文件名称
	if(m_file_dir.size()<=0)
	{
		cout<<"no file_dir is given!"<<endl;
		return false;
	}
	
	// 准备每个pcd文件的2D-RGB信息与Range图像信息
	//boost::shared_ptr<openni_wrapper::Image> openni_image;
	//boost::shared_ptr<openni_wrapper::DepthImage> openni_depth_image;

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Get OpenNI visual image
	static unsigned rgb_array_size = 0;
	static boost::shared_array<unsigned char> rgb_array(0);
	static unsigned char* rgb_buffer = 0;
	rgb_array_size = 480*640*3; // size of each frame
	rgb_array.reset(new unsigned char [rgb_array_size]);
	rgb_buffer = rgb_array.get();

	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;
	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	// 记录node个数
	static int n_nodes=0;

	// 记录SLAM时间
	double start_t_slam, start_t_fe;  // record SLAM time-consuming

	for(int i=0;i<num_pcds;i++)
	{
		stringstream strstream;
		strstream<<m_file_dir<<i+pcd_fst<<".pcd";
		strstream>>file_name;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);

		if(pcl::io::loadPCDFile(file_name,*m_tmpPC)==-1)			 // 读取点云数据
		{
			cout<<"failed to loadPCDFile: "<<file_name<<endl;
			return false;
		}
		
		if(_LOG){ 
			// record start time of SLAM
			start_t_slam = ::GetTickCount();
			// initialize log parameters
			gl_pf_gs=0; // graph_size
			gl_pf_fe=0;	// time for Feature Extraction
			gl_pf_fm=0; // time for Feature Match
			gl_pf_me=0; // time for Motion Estimation
			gl_pf_op=0; // time for Graph Optimization
			gl_pf_slam=0; // time for total Slam
		}
		getImagesandDepthMetaData(m_tmpPC,rgb_buffer,depth_buffer);
		// Copy RGB data from OpenNI image to cv mat image
		memcpy(cvRGBImage.data,rgb_buffer /*xnImd.Data()*/, 640*480*3);
		memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
		cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);

		if(_LOG) start_t_fe=::GetTickCount(); // 记录特征提取所需要的时间
		// 创建新的Pose节点
		Node* node_ptr = createFeatureNode(cvRGBImage, m_tmpPC, cvDepth8UC1Image);
		if(_LOG) gl_pf_fe=::GetTickCount() - start_t_fe;

		n_nodes++;
		// 加入到Hogman结构中
		if(!m_graph_mgr.addNode(node_ptr))
		{
			cout<<"false to add node: "<<n_nodes<<endl;
			delete node_ptr;
			continue;
			//return false;
		}
		else{
			cout<<"succeed to add node: "<<n_nodes<<endl;
			// 如果需要Log的话
			if(_LOG){
				gl_pf_slam=::GetTickCount()-start_t_slam;
				mylogfile.writeid(n_nodes-1); // id of frame
				double graphsize = (double)m_graph_mgr.graph_.size();
				mylogfile.writeintolog(graphsize); // graph size
				mylogfile.writeintolog(gl_pf_fe); // FE
				mylogfile.writeintolog(gl_pf_fm); // FM
				mylogfile.writeintolog(gl_pf_me); // R_ME
				mylogfile.writeintolog(gl_pf_op); // OP
				mylogfile.writeintolog(gl_pf_slam,true); // Total Slam
			}
		}
		file_name.clear();
	}
			
	// 输出融合之后的点云信息
	//m_graph_mgr.FuseOutPC(out_pc);
	m_graph_mgr.FuseOutPCVector(out_pc);
	SparsePC(out_pc,0.02); // 可能不需要
	return true;
}

// ASCII version
bool CFusePcd::compute_ORB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_pc)
{
	static int num_pcds=30; // 文件个数
	static int pcd_fst=1;	// 起始文件
	string file_name;		// 文件名称
	if(m_file_dir.size()<=0)
	{
		cout<<"no file_dir is given!"<<endl;
		return false;
	}

	// Create cv mat image with the right size
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvGrayImage(480, 640,CV_8UC1);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Get OpenNI visual image
	static boost::shared_array<unsigned char> rgb_array(0);
	rgb_array.reset(new unsigned char[480*640*3]);
	static unsigned char* rgb_buffer = rgb_array.get();

	// Get OpenNI depth image
	static boost::shared_array<unsigned char> depth_array(0);
	depth_array.reset(new unsigned char [480*640*2]);
	static unsigned char* depth_buffer = depth_array.get();

	//// 记录node个数
	static int n_nodes=0;

	// 记录SLAM时间
	double start_t_slam, start_t_fe;  // record SLAM time-consuming

	for(int i=0;i<num_pcds;i++)
	{
		stringstream strstream;
		strstream<<m_file_dir<<i+pcd_fst<<".pcd";
		strstream>>file_name;

		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > m_tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);

		if(pcl::io::loadPCDFile(file_name,*m_tmpPC)==-1)			 // 读取点云数据
		{
			cout<<"failed to loadPCDFile: "<<file_name<<endl;
			return false;
		}
		if(_LOG){ 
			// record start time of SLAM
			start_t_slam = ::GetTickCount();
			// initialize log parameters
			gl_pf_gs=0; // graph_size
			gl_pf_fe=0;	// time for Feature Extraction
			gl_pf_fm=0; // time for Feature Match
			gl_pf_me=0; // time for Motion Estimation
			gl_pf_op=0; // time for Graph Optimization
			gl_pf_slam=0; // time for total Slam
		}
		getImagesandDepthMetaData(m_tmpPC,rgb_buffer,depth_buffer);
		memcpy(cvRGBImage.data,rgb_buffer /*xnImd.Data()*/, 640*480*3);
		
		if(_LOG) start_t_fe=::GetTickCount(); // 记录特征提取所需要的时间
		// 创建新的ORB Node结点
		COrbNode* node_ptr = createFeatureNodeORB(cvRGBImage, m_tmpPC);
		n_nodes++;
		if(_LOG) gl_pf_fe=::GetTickCount() - start_t_fe;

		// 加入到Hogman结构中
		if(!m_graph_mgr.addORBNode(node_ptr))
		{
			cout<<"false to add node: "<<n_nodes+pcd_fst-1<<endl;
			delete node_ptr;
			continue;
			//return false;
		}
		else
		{
			cout<<"succeed to add node: "<<n_nodes+pcd_fst-1<<endl;
			// 如果需要Log的话
			if(_LOG){
				gl_pf_slam=::GetTickCount()-start_t_slam;
				mylogfile.writeid(n_nodes-1); // id of frame
				double graphsize = (double)m_graph_mgr.orb_graph_.size();
				mylogfile.writeintolog(graphsize); // graph size
				mylogfile.writeintolog(gl_pf_fe); // FE
				mylogfile.writeintolog(gl_pf_fm); // FM
				mylogfile.writeintolog(gl_pf_me); // R_ME
				mylogfile.writeintolog(gl_pf_op); // OP
				mylogfile.writeintolog(gl_pf_slam,true); // Total Slam
			}
		}
		file_name.clear();
	}

	// 输出融合之后的点云信息
	 m_graph_mgr.FuseOutPC_ORB(out_pc);
	 SparsePC(out_pc,0.02); // 可能不需要
	return true;
}

Node* CFusePcd::createFeatureNode(const cv::Mat& visual , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& point_cloud, 
									 const cv::Mat& depth){
	 Node* node_ptr = new Node(visual, m_detector, m_extractor, m_matcher,
		 point_cloud, depth);
	 return node_ptr;
}

COrbNode* CFusePcd::createFeatureNodeORB(const cv::Mat& visual , boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& point_cloud)
{
	COrbNode* node_ptr = new COrbNode(visual,point_cloud);
	return node_ptr;
}

void CFusePcd::getImagesandDepthMetaData(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud,
						  unsigned char* rgbbuf,
						  unsigned char* depthbuf
						  )
{
	unsigned short * pdepth =(unsigned short*) (depthbuf);
	unsigned char  * pimage = rgbbuf;  
	unsigned int totalnum = point_cloud->width * point_cloud->height;

	float bad_point = std::numeric_limits<float>::quiet_NaN();


	for(size_t i=0;i<totalnum;i++){

		pcl::PointXYZRGB& pt = point_cloud->points[i];
		// get rgb-info 
		*pimage = pt.r;
		pimage++;
		*pimage = pt.g;
		pimage++;
		*pimage = pt.b;
		pimage++;

		// get depth-info
		if(pt.x == bad_point && pt.y == bad_point && pt.z == bad_point){
			*pdepth = 0;
		}
		else
		{
			*pdepth = pt.z * 1000.0f;
		}
		pdepth ++;
	}
}