#include "globaldefinition.h"
#include "preheader.h"
#include "NarfNode.h"
#include <fstream>
#include "CPose3D.h"
#include "pcl/registration/transforms.h"
#include "boost/shared_array.hpp"
#include "node.h"
#include "opencv2/core/core.hpp"  
#include "opencv2/highgui/highgui.hpp" 

void testRoompcd()
{
	//string filename("D:\\MyProjects\\pcl\\RangeImage\\Debug\\room_scan2.pcd");
	string filename("D:\\MyProjects\\pcl\\conference_room\\conference_room\\cloud_000.pcd");
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pc(new pcl::PointCloud<pcl::PointXYZ>);
	if(pcl::io::loadPCDFile(filename,*m_pc)==-1)
	{
		cout<<"failed to load pcd!"<<endl;
		return;
	}
	DisplayPT(m_pc);
}

string get3ints(int num)
{
	stringstream retstream;
	string ret;
	if(num<0 || num >=1000)
	{
		cout<<"error exceed [0,1000) with num="<<num<<endl;
		return string("");
	}
	if(num<10)
	{
		retstream<<"00"<<num;
	}
	else if(num<100)
	{
		retstream<<"0"<<num;
	}
	else
		retstream<<num;
	retstream>>ret;
	return ret;
}

void testConferenceRoom(int num1,int num2)
{
	string pose_dir("D:\\MyProjects\\pcl\\conference_room\\conference_room");
	stringstream pose_file_name_stream1,pose_file_name_stream2;
	string pose_file_name1,pose_file_name2;
	float pose_info1[7],pose_info2[7];
	pose_file_name_stream1<<pose_dir<<"\\pose_"<<get3ints(num1)<<".txt";
	pose_file_name_stream2<<pose_dir<<"\\pose_"<<get3ints(num2)<<".txt";
	pose_file_name_stream1>>pose_file_name1;
	pose_file_name_stream2>>pose_file_name2;

	ifstream inf1(pose_file_name1.c_str());
	ifstream inf2(pose_file_name2.c_str());
	for(size_t i=0;i<7;i++)
	{
		inf1>>pose_info1[i];
		inf2>>pose_info2[i];
	}
	CPose3D p1(pose_info1);
	CPose3D p2(pose_info2);
	CPose3D transpose1=p2-p1;
	
	string pcdfiledir("D:\\MyProjects\\pcl\\conference_room\\conference_room");
	stringstream pcd_file_name_stream1,pcd_file_name_stream2;
	string pcd_file_name1,pcd_file_name2;
	pcd_file_name_stream1<<pcdfiledir<<"\\cloud_"<<get3ints(num1)<<".pcd";
	pcd_file_name_stream2<<pcdfiledir<<"\\cloud_"<<get3ints(num2)<<".pcd";
	pcd_file_name_stream1>>pcd_file_name1;
	pcd_file_name_stream2>>pcd_file_name2;

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pc1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pc2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_disc(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_temp(new pcl::PointCloud<pcl::PointXYZ>);

	if(pcl::io::loadPCDFile(pcd_file_name1,*m_pc1)==-1 || pcl::io::loadPCDFile(pcd_file_name2,*m_pc2)==-1)
	{
		cout<<"failed to load pcd!"<<endl;
		return;
	}
	Eigen::Matrix4f HM;
	transpose1.getHomogeneousMatrix(HM);
	pcl::transformPointCloud(*m_pc2,*m_temp,HM);
	m_disc->points.insert(m_disc->points.end(),m_pc1->points.begin(),m_pc1->points.end());
	m_disc->points.insert(m_disc->points.end(),m_temp->points.begin(),m_temp->points.end());
	DisplayPT(m_disc);

	/*cout<<"transpose1=p2-p1"<<endl;
	transpose1.output(std::cout);
	cout<<"transpose2=HM1.inverse()*HM2"<<endl;
	transpose2.output(std::cout);*/

	//FILE* pose_file=fopen(pose_file_name.c_str(),"rb");
	/*if(fread((void*)pose_info,sizeof(float),7,pose_file)!=7)
	{
		cout<<"failed to read all pose_info!"<<endl;
	}
	else
	{
		for(size_t i=0;i<7;i++)
			cout<<pose_info[i]<<" ";
		cout<<endl;
	}*/
	//fclose(pose_file);
}

//Transformation3 eigen2Hogman(const Eigen::Matrix4f& eigen_mat) {
//	std::clock_t starttime=std::clock();
//
//	Eigen::Affine3f eigen_transform(eigen_mat);
//	Eigen::Quaternionf eigen_quat(eigen_transform.rotation());
//	Vector3 translation(eigen_mat(0, 3), eigen_mat(1, 3), eigen_mat(2, 3));
//	Quaternion rotation(eigen_quat.x(), eigen_quat.y(), eigen_quat.z(),
//		eigen_quat.w());
//	Transformation3 result(translation, rotation);
//
//	return result;
//}

void findValidMatches(vector<cv::DMatch>* InitialMatches, vector<cv::DMatch>* ValidMatches,CPose3D& trans,
					  pcl::PointCloud<pcl::PointXYZ>::Ptr& m_srckpts,pcl::PointCloud<pcl::PointXYZ>::Ptr& m_tarkpts)
{
	cv::DMatch vmatch;
	Eigen::Matrix4f HM;
	trans.getHomogeneousMatrix(HM);
	Eigen::Vector4f src;
	Eigen::Vector4f dst;
	for(size_t i=0;i<InitialMatches->size();i++)
	{
		cv::DMatch& imatch=(*InitialMatches)[i];
		pcl::PointXYZ& sp1=m_srckpts->points[imatch.queryIdx];
		pcl::PointXYZ& sp2=m_tarkpts->points[imatch.trainIdx];
		pcl::PointXYZ sp3;
		
		// calculate transformed point from sp1->sp3
		src=sp1.getVector4fMap();
		dst=HM*src;
		
		sp3.x=dst[0]; sp3.y=dst[1]; sp3.z=dst[2];
		
		if(((CNarfNode*)0)->IsSamePoint(sp2,sp3))// this is valid match
		{
			vmatch=imatch;
			ValidMatches->push_back(vmatch);
		}
	}
}

void showMatchedFeatures(string file_dir,int num1,int num2,FeatureType feature_type,int _k)
{
	// 1, read ".pcd" file from disk
	string file_name1,file_name2;
	stringstream file_name_stream1,file_name_stream2;
	if(file_dir.find_last_of("\\")!=file_dir.size()-1)
	{
		file_dir.append("\\");
	}
	file_name_stream1<<file_dir<<"cloud_"<<get3ints(num1)<<".pcd";
	file_name_stream2<<file_dir<<"cloud_"<<get3ints(num2)<<".pcd";
	file_name_stream1>>file_name1;
	file_name_stream2>>file_name2;
	
	// 2, construct CNarfNode calculate PFH & NARF Features
	CNarfNode node_tar(file_name1.c_str());
	CNarfNode node_src(file_name2.c_str());

	// 3, find matched features using PFH or NARF or PFH+NARF
	const int k=_k;
	vector<cv::DMatch> matchedf;
	switch(feature_type)
	{
	case FeatureType::PFH:
		node_src.FindPairsFlann(&node_tar,&matchedf,false,k);// match using PFH
		break;
	case FeatureType::NARF:
		node_src.FindPairsFlann(&node_tar,&matchedf,true,k); // match using NARF
		break;
	case FeatureType::PFHNARF:
		node_src.FindPairsFlann2(&node_tar,&matchedf,k);	// match using both PFH & NARF
		break;
	default:
		break;
	}
	//node_src.FindPairsFlann(&node_tar,&matchedf,false);
	//node_src.FindMultiPairsFlann(&node_tar,&matchedf,2);
	//node_src.FindPairsFlann2(&node_tar,&matchedf,5);
	//node_src.DeleteMultiMatches(&matchedf);

	// RANSAC method to find actual matched pairs
	/*vector<cv::DMatch> inliers;
	float rmse;
	Eigen::Matrix4f resulting_transformation;
	bool bInliers = node_src.getRelativeTransformationTo(&node_tar,&matchedf, resulting_transformation, rmse, inliers);
	if(bInliers==false)
	{
		cout<<"failed to find matches using RANSAC!"<<endl;
	}*/
	
	// 4, show point cloud and Features
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1=node_tar.m_pPC;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2=node_src.m_pPC;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_left(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_right(new pcl::PointCloud<pcl::PointXYZRGB>);


	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints1_back=node_tar.m_pfhKeyPoints;//(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints2_back=node_src.m_pfhKeyPoints;//(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_left(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_right(new pcl::PointCloud<pcl::PointXYZ>);

	// Shift the first clouds' points to the left
	//const Eigen::Vector3f translate (0.0, 0.0, 0.3);
	const Eigen::Vector3f translate (2.0, 0.0, 0.0);
	const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
	pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
	pcl::transformPointCloud (*keypoints1_back, *keypoints_left, -translate, no_rotation);

	// Shift the second clouds' points to the right
	pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
	pcl::transformPointCloud (*keypoints2_back, *keypoints_right, translate, no_rotation);

	// Add the clouds to the vizualizer
	pcl::visualization::PCLVisualizer viz;
	viz.setBackgroundColor (1, 1, 1);
	viz.addCoordinateSystem (1.0f);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler_left (points_left, 0, 0, 0);
	viz.addPointCloud (points_left, point_cloud_color_handler_left, "target point cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> point_cloud_color_handler_right (points_right, 0, 0, 0);
	viz.addPointCloud (points_right, point_cloud_color_handler_right, "source point cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler_left (keypoints_left, 0, 255, 0);
	viz.addPointCloud<pcl::PointXYZ> (keypoints_left, keypoints_color_handler_left, "keypoints_left");
	viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints_left");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler_right (keypoints_right, 255, 0, 0);
	viz.addPointCloud<pcl::PointXYZ> (keypoints_right, keypoints_color_handler_right, "keypoints_right");
	viz.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints_right");

	// 5, read "pose.txt" to get groundtruth transformation
	stringstream pose_file_name_stream1,pose_file_name_stream2;
	string pose_file_name1,pose_file_name2;
	float pose_info1[7],pose_info2[7];
	pose_file_name_stream1<<file_dir<<"\\pose_"<<get3ints(num1)<<".txt";
	pose_file_name_stream2<<file_dir<<"\\pose_"<<get3ints(num2)<<".txt";
	pose_file_name_stream1>>pose_file_name1;
	pose_file_name_stream2>>pose_file_name2;

	ifstream inf1(pose_file_name1.c_str());
	ifstream inf2(pose_file_name2.c_str());
	//for(size_t i=0;i<7;i++)
	//{
	//	inf1>>pose_info1[i];
	//	inf2>>pose_info2[i];
	//}
	//CPose3D p1(pose_info1);
	//CPose3D p2(pose_info2);
	//CPose3D transpose1=p2-p1;

	CPose3D p1;
	CPose3D p2;
	p1.input_f(inf1);
	p2.input_f(inf2);
	CPose3D transpose1=p2-p1;

	// 6, find valid matches from initial matches
	vector<cv::DMatch> valid_match;
	findValidMatches(&matchedf,&valid_match,transpose1,node_src.m_pfhKeyPoints,node_tar.m_pfhKeyPoints);
	
	cout<<"find valid matches: "<<valid_match.size()<<" in total matches: "<<matchedf.size()<<endl;

	// draw initial matches with red line
	//for(size_t i=0;i<matchedf.size();i++)
	//{
	//	const pcl::PointXYZ & p_left = keypoints_left->points[matchedf[i].trainIdx];
	//	const pcl::PointXYZ & p_right = keypoints_right->points[matchedf[i].queryIdx];
	//	// Generate a unique string for each line
	//	std::stringstream ss ("redline");
	//	ss << i;

	//	// Draw the line
	//	viz.addLine (p_left, p_right, 1.0, 0.0, 0.0, ss.str ());
	//}
	// draw valid matches with green line
	for(size_t i=0;i<valid_match.size();i++)
	{
		const pcl::PointXYZ & p_left = keypoints_left->points[valid_match[i].trainIdx];
		const pcl::PointXYZ & p_right = keypoints_right->points[valid_match[i].queryIdx];
		// Generate a unique string for each line
		std::stringstream ss ("greenline");
		ss << i;

		// Draw the line
		viz.addLine (p_left, p_right, 0.0, 1.0, 0.0, ss.str ());
	}

	// Give control over to the visualizer
	viz.spin ();
}

pair<double,int> getValidGrade(string file_dir,int num1,int num2,FeatureType feature_type,int _k)
{
	// 1, read ".pcd" file from disk
	string file_name1,file_name2;
	stringstream file_name_stream1,file_name_stream2;
	if(file_dir.find_last_of("\\")!=file_dir.size()-1)
	{
		file_dir.append("\\");
	}
	file_name_stream1<<file_dir<<"cloud_"<<get3ints(num1)<<".pcd";
	file_name_stream2<<file_dir<<"cloud_"<<get3ints(num2)<<".pcd";
	file_name_stream1>>file_name1;
	file_name_stream2>>file_name2;

	// 2, construct CNarfNode calculate PFH & NARF Features
	CNarfNode node_tar(file_name1.c_str());
	if(!node_tar.m_ValidNode)
		return make_pair(-1.0,0);
	CNarfNode node_src(file_name2.c_str());
	if(!node_src.m_ValidNode)
		return make_pair(-1.0,0);

	// 3, find matched features using PFH or NARF or PFH+NARF
	const int k=_k;
	vector<cv::DMatch> matchedf;
	switch(feature_type)
	{
	case FeatureType::PFH:
		node_src.FindPairsFlann(&node_tar,&matchedf,false,k);// match using PFH
		break;
	case FeatureType::NARF:
		node_src.FindPairsFlann(&node_tar,&matchedf,true,k); // match using NARF
		break;
	case FeatureType::PFHNARF:
		node_src.FindPairsFlann2(&node_tar,&matchedf,k);	// match using both PFH & NARF
		break;
	default:
		break;
	}
	// no feature is matched
	if(matchedf.size()==0)
	{
		cout<<"Features not matched!"<<endl;
		return make_pair(-1.0,0);
	}

	// 5, read "pose.txt" to get groundtruth transformation
	stringstream pose_file_name_stream1,pose_file_name_stream2;
	string pose_file_name1,pose_file_name2;
	float pose_info1[7],pose_info2[7];
	pose_file_name_stream1<<file_dir<<"\\pose_"<<get3ints(num1)<<".txt";
	pose_file_name_stream2<<file_dir<<"\\pose_"<<get3ints(num2)<<".txt";
	pose_file_name_stream1>>pose_file_name1;
	pose_file_name_stream2>>pose_file_name2;

	ifstream inf1(pose_file_name1.c_str());
	ifstream inf2(pose_file_name2.c_str());
	for(size_t i=0;i<7;i++)
	{
		inf1>>pose_info1[i];
		inf2>>pose_info2[i];
	}
	CPose3D p1(pose_info1);
	CPose3D p2(pose_info2);
	CPose3D transpose1=p2-p1;

	// 6, find valid matches from initial matches
	vector<cv::DMatch> valid_match;
	findValidMatches(&matchedf,&valid_match,transpose1,node_src.m_pfhKeyPoints,node_tar.m_pfhKeyPoints);

	double ret1=(double)((double)(valid_match.size())/(double)(matchedf.size()));

	return make_pair(ret1,matchedf.size());
}

void testConferenceRoomSet()
{
	string file_dir("D:\\MyProjects\\pcl\\conference_room\\conference_room");
	//string record_file_name("D:\\MyProjects\\pcl\\conference_room\\NARF");
	//string record_file_name("D:\\MyProjects\\pcl\\conference_room\\PFHNARF");
	string record_file_name("D:\\MyProjects\\pcl\\conference_room\\PFH");

	pair<double,int> match_grade;
	int total_item=348;
	int random_times=20;
	int valid_times=0;
	
	// for each k, calculate valid ratio and total matched pairs
	float total_grade;
	int total_num;

	stringstream record_all_k;
	record_all_k<<record_file_name<<".txt";
	string tmp;
	record_all_k>>tmp;
	ofstream record_all_k_file(tmp.c_str()/*record_all_k.str()*/);
	record_all_k_file<<"K"<<"     "<<"Valid_Grade"<<"     "<<"Total Num"<<endl;

	srand((long)std::clock());

	for(int k=1;k<=10;k++)
	{
		stringstream record_file_name_stream;
		record_file_name_stream<<record_file_name<<"\\"<<k<<".txt";
		string tmp1;
		record_file_name_stream>>tmp1;
		ofstream record_file(tmp1.c_str());
		total_grade=0;
		valid_times=0;
		total_num=0;
		for(int i=0;i<=random_times;i++)
		{
			int index_pose=rand()%total_item;
			//match_grade=getValidGrade(file_dir,index_pose,index_pose+1,FeatureType::NARF,k);
			//match_grade=getValidGrade(file_dir,index_pose,index_pose+1,FeatureType::PFHNARF,k);
			match_grade=getValidGrade(file_dir,index_pose,index_pose+1,FeatureType::PFH,k);
			if(match_grade.first>=0)
			{
				record_file<<match_grade.first<<"     "<<match_grade.second<<endl;
				total_grade+=match_grade.first;
				total_num+=match_grade.second;
				valid_times++;
			}
			else
			{
				i--;
			}
		}
		total_grade/=valid_times;
		total_num/=valid_times;
		record_all_k_file<<k<<"     "<<total_grade<<"     "<<total_num<<endl;
	}
}

void getImagesandDepthMetaData(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud,
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

void motionEstimationPCD(string dir)
{
	// 1, read pcds from disk
	string srcf("D:\\MyProjects\\pcl\\RangeImage\\Debug\\cloud_002.pcd");
	string dstf("D:\\MyProjects\\pcl\\RangeImage\\Debug\\cloud_001.pcd");

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> dst_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::io::loadPCDFile(srcf,*src_cloud);
	pcl::io::loadPCDFile(dstf,*dst_cloud);

	// 2,create cv mat image
	cv::Mat cvRGBImage(480, 640, CV_8UC3);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Get OpenNI visual image
	static boost::shared_array<unsigned char> src_rgb_array(0);
	src_rgb_array.reset(new unsigned char[480*640*3]);
	static unsigned char* src_rgb_buffer = src_rgb_array.get();

	static boost::shared_array<unsigned char> dst_rgb_array(0);
	dst_rgb_array.reset(new unsigned char[480*640*3]);
	static unsigned char* dst_rgb_buffer = dst_rgb_array.get();

	// Get OpenNI depth image
	static unsigned depth_array_size = 0;
	static boost::shared_array<unsigned char> depth_array(0);
	static unsigned char* depth_buffer = 0;

	depth_array_size = 480*640*2;
	depth_array.reset(new unsigned char [depth_array_size]);
	depth_buffer = depth_array.get();

	//  
	cv::Ptr<cv::FeatureDetector> m_detector;
	cv::Ptr<cv::DescriptorExtractor> m_extractor;
	cv::Ptr<cv::DescriptorMatcher > m_matcher;

	m_detector = new cv::DynamicAdaptedFeatureDetector(new cv::SurfAdjuster(),
		global_adjuster_min_keypoints,
		global_adjuster_max_keypoints,
		global_surf_adjuster_max_iterations);

	m_extractor = cv::DescriptorExtractor::create("SURF");
	m_matcher = cv::DescriptorMatcher::create("BruteForce");

	// 3, generate Node src and dst
	getImagesandDepthMetaData(src_cloud,src_rgb_buffer, depth_buffer);
	memcpy(cvRGBImage.data,src_rgb_buffer, 640*480*3);
	memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
	cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);

	Node* src_ptr = new Node(cvRGBImage, m_detector, m_extractor, m_matcher,src_cloud, cvDepth8UC1Image);
	src_ptr->buildFlannIndex();

	getImagesandDepthMetaData(dst_cloud,dst_rgb_buffer, depth_buffer);
	memcpy(cvRGBImage.data,dst_rgb_buffer, 640*480*3);
	memcpy(cvDepthImage.data, depth_buffer, 640*480*2);
	cvDepthImage.convertTo(cvDepth8UC1Image, CV_8UC1);

	Node* dst_ptr = new Node(cvRGBImage, m_detector, m_extractor, m_matcher,dst_cloud, cvDepth8UC1Image);
	dst_ptr->buildFlannIndex();

	// Match those two nodes
	MatchingResult mr = src_ptr->matchNodePair(dst_ptr);
	
	CPose3D trans(mr.final_trafo);
	trans.output(std::cout);
	
	// write trans info to txt
	stringstream record;
	record<<dir<<"\\"<<"cloud_001.txt";
	string tmp;
	record>>tmp;
	ofstream dst_record(tmp.c_str());
	CPose3D dst;
	dst.output(dst_record);

	stringstream record1;
	record1<<dir<<"\\"<<"cloud_002.txt";
	record1>>tmp;
	ofstream src_record(tmp.c_str());
	CPose3D src;
	src=dst+trans;
	src.output(src_record);
}

void calPCfromImageAndDepth(cv::Mat& image, cv::Mat& depth, \
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

void writef(string file_f,cv::Mat& image)
{
	ofstream outf(file_f.c_str());
	if(outf.is_open()){

		for(int i=0;i<image.rows;i++)
		{
			for(int j=0;j<image.cols;j++)
			{
				outf<<image.at<unsigned short>(i,j)<<" ";
			}
			outf<<endl;
		}
		outf.close();
	}
	else
	{
		cout<<"failed to open file: "<<file_f<<endl;
	}
	return ;
}

// 测试所抓取的image 与 depth是否匹配
void showBitNodes()
{
	string file_dir("D:\\MyProjects\\KinectDeviation\\cornerloc\\");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i=2;i<=34;i++)
	{
		// 获取文件名
		stringstream imfs,defs;
		imfs<<file_dir<<"i"<<i<<".png";
		defs<<file_dir<<"d"<<i<<".png";

		string imf,def;
		imfs>>imf;
		defs>>def;
		
		// 读取数据
		cv::Mat image;
		cv::Mat depth;
		image=cv::imread(imf);
		depth=cv::imread(def,2);

		// 合成点云数据
		calPCfromImageAndDepth(image,depth,m_tmpPC);
		DisplayPT(m_tmpPC);
	}
}