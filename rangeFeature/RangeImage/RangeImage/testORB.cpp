#include "preheader.h"
#include "globaldefinition.h"
#include "ORBNode.h"
#include "node.h"
#include "boost/shared_array.hpp"
#include "opencv2/gpu/gpu.hpp"
#include "FusePCD.h"

void testORBPCD(string dstf, string srcf)
{
	cout<<"Using ORB Match: "<<endl;
	testORBMatch(dstf,srcf);
	cout<<"Using SURF Match: "<<endl;
	motionEstimationPCD(dstf,srcf);
}


void testORBMatch(string file_dir)
{
	int tmpv=5;
	for(int file_index=tmpv;file_index<tmpv+1;file_index++)
	{
		stringstream fs1,fs2;
		string f1,f2;
		fs1<<file_dir<<file_index<<".pcd";
		fs2<<file_dir<<file_index+1<<".pcd";
		fs1>>f1;
		fs2>>f2;
		cout<<"pairs: "<<file_index<<" and "<<file_index+1<<endl;
		//testORBMatchShow(f1,f2);
		testSURFMatchShow(f1,f2);
	}
}


void testSURFMatchShow(string dstf, string srcf)
{
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > dst_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::io::loadPCDFile(srcf,*src_cloud);
	pcl::io::loadPCDFile(dstf,*dst_cloud);

	// 2,create cv mat image
	cv::Mat cvRGBImageSrc(480, 640, CV_8UC3);
	cv::Mat cvRGBImageDst(480, 640, CV_8UC3);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Get OpenNI visual image
	static boost::shared_array<unsigned char> src_rgb_array(0);
	src_rgb_array.reset(new unsigned char[480*640*3]);
	unsigned char* src_rgb_buffer = src_rgb_array.get();

	static boost::shared_array<unsigned char> dst_rgb_array(0);
	dst_rgb_array.reset(new unsigned char[480*640*3]);
	unsigned char* dst_rgb_buffer = dst_rgb_array.get();

	// Get OpenNI depth image
	static boost::shared_array<unsigned char> depth_array(0);
	depth_array.reset(new unsigned char [480*640*2]);
	unsigned char* depth_buffer = depth_array.get();

	// OpenCV 建立结点的参数
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
	memcpy(cvRGBImageSrc.data,src_rgb_buffer, 640*480*3);
	memcpy(cvDepthImage.data,depth_buffer,640*480*2);
	cvDepthImage.convertTo(cvDepth8UC1Image,CV_8UC1);
	Node* src_ptr = new Node(cvRGBImageSrc, m_detector, m_extractor, m_matcher,src_cloud, cvDepth8UC1Image);
	src_ptr->buildFlannIndex();

	getImagesandDepthMetaData(dst_cloud,dst_rgb_buffer, depth_buffer);
	memcpy(cvRGBImageDst.data,dst_rgb_buffer, 640*480*3);
	memcpy(cvDepthImage.data,depth_buffer,640*480*2);
	cvDepthImage.convertTo(cvDepth8UC1Image,CV_8UC1);
	Node* dst_ptr = new Node(cvRGBImageDst, m_detector, m_extractor, m_matcher,dst_cloud, cvDepth8UC1Image);
	dst_ptr->buildFlannIndex();

	// 找到两个Node中的匹配的特征点对，
	//vector<cv::DMatch> matches;  
	//src_ptr->findPairsBruteForce(dst_ptr,matches);

	// Match those two nodes
	MatchingResult mr = src_ptr->matchNodePair(dst_ptr);

	// 显示转换矩阵
	// CPose3D trans(mr.final_trafo);
	//trans.output(std::cout);

	// 获取匹配的特征点下标
	//vector<int> src_dics,dst_dics;
	//std::vector<cv::DMatch>& final_match = mr.inlier_matches;
	//for(int i=0;i<3/*final_match.size()*/;i++)
	//{
	//	cv::DMatch& cur_match=final_match[i];
	//	src_dics.push_back(cur_match.queryIdx);
	//	dst_dics.push_back(cur_match.trainIdx);
	//}
	//
	////pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_srcPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	////pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_srcfPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_dstfPC(new pcl::PointCloud<pcl::PointXYZRGB>);

	//// 获取特征点坐标
	//src_ptr->getFeatureLoc(src_dics,m_srcfPC);
	//dst_ptr->getFeatureLoc(dst_dics,m_dstfPC);

	//// 显示特征点在原点云中的位置
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	//AddPT2Viewer(viewer1,dst_ptr->m_pPC,"dst_raw_PC");
	//AddPT2Viewer(viewer1,m_dstfPC,"dst_featurePC",7);
	//showViewer(viewer1);

	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	//AddPT2Viewer(viewer2,src_ptr->m_pPC,"src_raw_PC");
	//AddPT2Viewer(viewer2,m_srcfPC,"src_featurePC",7);
	//showViewer(viewer2);

	//// 显示这些点对与图像
	cv::Mat img_matches;  
	cv::drawMatches(cvRGBImageSrc, src_ptr->feature_locations_2d_, cvRGBImageDst, dst_ptr->feature_locations_2d_,  
		mr.inlier_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),  
		vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  
	cv::imshow( "SURF_Match", img_matches);  
	cvWaitKey(); 

	// 显示配准后的3D点云图
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_srcPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::transformPointCloud(*(src_ptr->m_pPC),*m_disPC,mr.final_trafo);
	m_disPC->points.insert(m_disPC->points.end(),dst_ptr->m_pPC->points.begin(),dst_ptr->m_pPC->points.end());
	//SparsePC(m_disPC,0.04);
	DisplayPT(m_disPC);

	delete src_ptr;
	delete dst_ptr;

}


void testORBMatchShow(string dstf, string srcf)
{
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > dst_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::io::loadPCDFile(srcf,*src_cloud);
	pcl::io::loadPCDFile(dstf,*dst_cloud);

	// 2,create cv mat image
	cv::Mat cvRGBImageSrc(480, 640, CV_8UC3);
	cv::Mat cvRGBImageDst(480, 640, CV_8UC3);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Get OpenNI visual image
	static boost::shared_array<unsigned char> src_rgb_array(0);
	src_rgb_array.reset(new unsigned char[480*640*3]);
	unsigned char* src_rgb_buffer = src_rgb_array.get();

	static boost::shared_array<unsigned char> dst_rgb_array(0);
	dst_rgb_array.reset(new unsigned char[480*640*3]);
	unsigned char* dst_rgb_buffer = dst_rgb_array.get();

	// Get OpenNI depth image
	static boost::shared_array<unsigned char> depth_array(0);
	depth_array.reset(new unsigned char [480*640*2]);
	unsigned char* depth_buffer = depth_array.get();

	// 3, generate Node src and dst
	getImagesandDepthMetaData(src_cloud,src_rgb_buffer, depth_buffer);
	memcpy(cvRGBImageSrc.data,src_rgb_buffer, 640*480*3);

	getImagesandDepthMetaData(dst_cloud,dst_rgb_buffer, depth_buffer);
	memcpy(cvRGBImageDst.data,dst_rgb_buffer, 640*480*3);

	COrbNode* src_ptr = new COrbNode(cvRGBImageSrc,src_cloud);
	COrbNode* dst_ptr = new COrbNode(cvRGBImageDst,dst_cloud);
	
	// Match those two nodes
	MatchingResult mr = src_ptr->matchNodePair(dst_ptr);

	// 做10次随即选取点计算结果变化
	//srand((unsigned int)(time(NULL)));
	//for(int t=0;t<10;t++)
	//{
	//	// 随即获取三个点对
	//	vector<cv::DMatch> tmpMatch;
	//	vector<bool> Isused(mr.inlier_matches.size(),false);
	//	for(int i=0;i<3;)
	//	{
	//		int index = rand()%mr.inlier_matches.size();
	//		if(Isused[index]) continue;
	//		tmpMatch.push_back(mr.inlier_matches[index]);
	//		Isused[index]=true;
	//		i++;
	//	}
	//	// 利用这三个点对计算旋转矩阵
	//	vector<pcl::TMatchingPair> cors;
	//	for(int i=0;i<3;i++){
	//		Eigen::Vector4f src=src_ptr->m_feature_locations_3d_[tmpMatch[i].queryIdx];
	//		Eigen::Vector4f dst=dst_ptr->m_feature_locations_3d_[tmpMatch[i].trainIdx];
	//		cors.push_back(pcl::TMatchingPair(0,0,dst[0],dst[1],dst[2],src[0],src[1],src[2]));
	//	}
	//	pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
	//	CPose3D finalPose;
	//	icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
	//	cout<<t+1<<"th :"<<endl;
	//	finalPose.output(std::cout);

	//	// 显示这些点对与图像
	//	cv::Mat img_matches;  
	//	cv::drawMatches(cvRGBImageSrc, src_ptr->m_keyPoints, cvRGBImageDst, dst_ptr->m_keyPoints,  
	//		tmpMatch/*mr.inlier_matches*/, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),  
	//		vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  
	//	cv::imshow( "ORB_Match", img_matches);  
	//	cvWaitKey(); 
	//}
	//// 显示这些点对与图像
	//cv::Mat img_matches;  
	//cv::drawMatches(cvRGBImageSrc, src_ptr->m_keyPoints, cvRGBImageDst, dst_ptr->m_keyPoints,  
	//	mr.inlier_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),  
	//	vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  
	//cv::imshow( "Match", img_matches);  
	//cvWaitKey(); 

	CPose3D tras(mr.final_trafo);
	tras.output(std::cout);

	// 显示这些点对与图像
	cv::Mat img_matches;  
	cv::drawMatches(cvRGBImageSrc, src_ptr->m_keyPoints, cvRGBImageDst, dst_ptr->m_keyPoints,  
		mr.inlier_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),  
		vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  
	cv::imshow( "Match", img_matches);  
	cvWaitKey(); 

	// 获取匹配的特征点下标
	//vector<int> src_dics,dst_dics;
	//std::vector<cv::DMatch>& final_match = mr.inlier_matches;
	//for(int i=0;i<final_match.size();i++)
	//{
	//	cv::DMatch& cur_match=final_match[i];
	//	src_dics.push_back(cur_match.queryIdx);
	//	dst_dics.push_back(cur_match.trainIdx);
	//}

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_srcPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_disPC(new pcl::PointCloud<pcl::PointXYZRGB>);

	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_srcfPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_dstfPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	//
	//// 获取特征点坐标
	//src_ptr->getFeatureLoc(src_dics,m_srcfPC);
	//dst_ptr->getFeatureLoc(dst_dics,m_dstfPC);

	//// 显示特征点在原点云中的位置
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	//AddPT2Viewer(viewer1,dst_ptr->m_pPC,"dst_raw_PC");
	//AddPT2Viewer(viewer1,m_dstfPC,"dst_featurePC",7);
	//showViewer(viewer1);
	//
	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2 (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	//AddPT2Viewer(viewer2,src_ptr->m_pPC,"src_raw_PC");
	//AddPT2Viewer(viewer2,m_srcfPC,"src_featurePC",7);
	//showViewer(viewer2);
	//
	//pcl::transformPointCloud(*(src_ptr->m_pPC),*m_disPC,mr.final_trafo);
	//m_disPC->points.insert(m_disPC->points.end(),dst_ptr->m_pPC->points.begin(),dst_ptr->m_pPC->points.end());
	//DisplayPT(m_disPC);

	delete src_ptr;
	delete dst_ptr;
}

void testORBMatch(string dstf, string srcf)
{
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > dst_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::io::loadPCDFile(srcf,*src_cloud);
	pcl::io::loadPCDFile(dstf,*dst_cloud);

	// 2,create cv mat image
	cv::Mat cvRGBImageSrc(480, 640, CV_8UC3);
	cv::Mat cvRGBImageDst(480, 640, CV_8UC3);
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
	static boost::shared_array<unsigned char> depth_array(0);
	depth_array.reset(new unsigned char [480*640*2]);
	static unsigned char* depth_buffer = depth_array.get();
	
	// 3, generate Node src and dst
	getImagesandDepthMetaData(src_cloud,src_rgb_buffer, depth_buffer);
	memcpy(cvRGBImageSrc.data,src_rgb_buffer, 640*480*3);

	getImagesandDepthMetaData(dst_cloud,dst_rgb_buffer, depth_buffer);
	memcpy(cvRGBImageDst.data,dst_rgb_buffer, 640*480*3);

	COrbNode* src_ptr = new COrbNode(cvRGBImageSrc,src_cloud);
	COrbNode* dst_ptr = new COrbNode(cvRGBImageDst,dst_cloud);

	// Match those two nodes
	MatchingResult mr = src_ptr->matchNodePair(dst_ptr);

	CPose3D trans(mr.final_trafo);
	trans.output(std::cout);
	delete src_ptr;
	delete dst_ptr;
}

// 测试不同的点对计算出的旋转矩阵是否一致
void testMatchMatrix(string dstf, string srcf)
{
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > dst_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::io::loadPCDFile(srcf,*src_cloud);
	pcl::io::loadPCDFile(dstf,*dst_cloud);

	// 2,create cv mat image
	cv::Mat cvRGBImageSrc(480, 640, CV_8UC3);
	cv::Mat cvRGBImageDst(480, 640, CV_8UC3);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Get OpenNI visual image
	static boost::shared_array<unsigned char> src_rgb_array(0);
	src_rgb_array.reset(new unsigned char[480*640*3]);
	unsigned char* src_rgb_buffer = src_rgb_array.get();

	static boost::shared_array<unsigned char> dst_rgb_array(0);
	dst_rgb_array.reset(new unsigned char[480*640*3]);
	unsigned char* dst_rgb_buffer = dst_rgb_array.get();

	// Get OpenNI depth image
	static boost::shared_array<unsigned char> depth_array(0);
	depth_array.reset(new unsigned char [480*640*2]);
	unsigned char* depth_buffer = depth_array.get();

	// OpenCV 建立结点的参数
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
	memcpy(cvRGBImageSrc.data,src_rgb_buffer, 640*480*3);
	memcpy(cvDepthImage.data,depth_buffer,640*480*2);
	cvDepthImage.convertTo(cvDepth8UC1Image,CV_8UC1);
	Node* src_ptr = new Node(cvRGBImageSrc, m_detector, m_extractor, m_matcher,src_cloud, cvDepth8UC1Image);
	src_ptr->buildFlannIndex();

	getImagesandDepthMetaData(dst_cloud,dst_rgb_buffer, depth_buffer);
	memcpy(cvRGBImageDst.data,dst_rgb_buffer, 640*480*3);
	memcpy(cvDepthImage.data,depth_buffer,640*480*2);
	cvDepthImage.convertTo(cvDepth8UC1Image,CV_8UC1);
	Node* dst_ptr = new Node(cvRGBImageDst, m_detector, m_extractor, m_matcher,dst_cloud, cvDepth8UC1Image);
	dst_ptr->buildFlannIndex();

	// 找到两个Node中的匹配的特征点对，
	//vector<cv::DMatch> matches;  
	//src_ptr->findPairsBruteForce(dst_ptr,matches);

	// Match those two nodes
	MatchingResult mr = src_ptr->matchNodePair(dst_ptr);

	CPose3D surf_trans(mr.final_trafo);
	cout<<"using SURF: "<<endl;
	surf_trans.output(std::cout);

	COrbNode* src_ptr1=new COrbNode(cvRGBImageSrc,src_cloud);
	COrbNode* dst_ptr1=new COrbNode(cvRGBImageDst,dst_cloud);
	MatchingResult mr1 = src_ptr1->matchNodePair(dst_ptr1);

	CPose3D orb_trans(mr1.final_trafo);
	cout<<"using ORB: "<<endl;
	orb_trans.output(std::cout);

	Eigen::Vector4f src;
	Eigen::Vector4f dst;
	vector<cv::DMatch>& orb_match=mr1.inlier_matches;

	vector<pcl::TMatchingPair> cors;
	for(int i=0;i<3;i++)
	{
		cv::DMatch& curm=orb_match[i];
		src=src_ptr1->m_feature_locations_3d_[curm.queryIdx];
		dst=mr.final_trafo*src;
		cout<<"src:("<<src[0]<<","<<src[1]<<","<<src[2]<<"),dst:"<<dst[0]<<","<<dst[1]<<","<<dst[2]<<")"<<endl;
		cors.push_back(pcl::TMatchingPair(0,0,dst[0]+rand_noise(),dst[1]+rand_noise(),dst[2]+rand_noise(),\
			src[0]+rand_noise(),src[1]+rand_noise(),src[2]+rand_noise()));
	}

	pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
	CPose3D finalPose;
	icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
	cout<<"after using standard matched points:"<<endl;
	finalPose.output(std::cout);

}

void testFixMatch(string dstf,string srcf)
{
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > src_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > dst_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	//pcl::io::loadPCDFile(srcf,*src_cloud);
	//pcl::io::loadPCDFile(dstf,*dst_cloud);

	// 2,create cv mat image
	cv::Mat cvRGBImageSrc(480, 640, CV_8UC3);
	cv::Mat cvRGBImageDst(480, 640, CV_8UC3);
	cv::Mat cvDepthImage(480, 640, CV_16UC1);
	cv::Mat cvDepth8UC1Image(480, 640, CV_8UC1);

	// Get OpenNI visual image
	static boost::shared_array<unsigned char> src_rgb_array(0);
	src_rgb_array.reset(new unsigned char[480*640*3]);
	unsigned char* src_rgb_buffer = src_rgb_array.get();

	static boost::shared_array<unsigned char> dst_rgb_array(0);
	dst_rgb_array.reset(new unsigned char[480*640*3]);
	unsigned char* dst_rgb_buffer = dst_rgb_array.get();

	// Get OpenNI depth image
	static boost::shared_array<unsigned char> depth_array(0);
	depth_array.reset(new unsigned char [480*640*2]);
	unsigned char* depth_buffer = depth_array.get();

	// OpenCV 建立结点的参数
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
//	getImagesandDepthMetaData(src_cloud,src_rgb_buffer, depth_buffer);
//	memcpy(cvRGBImageSrc.data,src_rgb_buffer, 640*480*3);
//	memcpy(cvDepthImage.data,depth_buffer,640*480*2);
//	cvDepthImage.convertTo(cvDepth8UC1Image,CV_8UC1);
//	Node* src_ptr = new Node(cvRGBImageSrc, m_detector, m_extractor, m_matcher,src_cloud, cvDepth8UC1Image);
//	src_ptr->buildFlannIndex();
//
//	getImagesandDepthMetaData(dst_cloud,dst_rgb_buffer, depth_buffer);
//	memcpy(cvRGBImageDst.data,dst_rgb_buffer, 640*480*3);
//	memcpy(cvDepthImage.data,depth_buffer,640*480*2);
//	cvDepthImage.convertTo(cvDepth8UC1Image,CV_8UC1);
//	Node* dst_ptr = new Node(cvRGBImageDst, m_detector, m_extractor, m_matcher,dst_cloud, cvDepth8UC1Image);
//	dst_ptr->buildFlannIndex();
//
//	// 找到两个Node中的匹配的特征点对，
//	//vector<cv::DMatch> matches;  
//	//src_ptr->findPairsBruteForce(dst_ptr,matches);
//
//	// Match those two nodes
//	MatchingResult mr = src_ptr->matchNodePair(dst_ptr);
	
	vector<pcl::TMatchingPair> cors;
	cors.push_back(pcl::TMatchingPair(0,0,-0.0697162,0.524922,2.153,\
		-0.277181,0.529905,2.14));
	cors.push_back(pcl::TMatchingPair(0,0,0.257966,0.615497,2.376,\
		0.0273371,0.619642,2.392));
	cors.push_back(pcl::TMatchingPair(0,0,-0.378274,0.430194,1.947,\
		-0.569029,0.437714,1.915));
	pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
	CPose3D finalPose;
	icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
	cout<<"using ORB matched points:"<<endl;
	finalPose.output(std::cout);
	
	cors.clear();
	cors.push_back(pcl::TMatchingPair(0,0,-0.0532107,0.520158,2.15959,\
		-0.277181,0.529905,2.14));
	cors.push_back(pcl::TMatchingPair(0,0,0.276154,0.605991,2.37968,\
		0.0273371,0.619642,2.392));
	cors.push_back(pcl::TMatchingPair(0,0,-0.367244,0.431617,1.96506,\
		-0.569029,0.437714,1.915));
	/*cout<<"using SURF matched points: "<<endl;
	CPose3D trans(mr.final_trafo);
	trans.output(std::cout);
	for(int i=0;i<3;i++)
	{
		Eigen::Vector4f src,dst;
		src[0] = cors[i].other_x; src[1]=cors[i].other_y; src[2]=cors[i].other_z;
		dst = mr.final_trafo*src;
		cors[i].this_x = dst[0]; cors[i].this_y=dst[1]; cors[i].this_z=dst[2];
		cout<<"src:("<<src[0]<<","<<src[1]<<","<<src[2]<<"),dst:"<<dst[0]<<","<<dst[1]<<","<<dst[2]<<")"<<endl;
	}*/
	CPose3D finalPose1;
	icp.leastSquareErrorRigidTransformation6D(cors,finalPose1);
	cout<<"after using transformed matched points:"<<endl;
	finalPose1.output(std::cout);
}

void testMatrix()
{
	vector<pcl::TMatchingPair> cors;
	cors.push_back(pcl::TMatchingPair(0,0,-0.0697162,0.524922,2.153,\
		-0.277181,0.529905,2.14));
	cors.push_back(pcl::TMatchingPair(0,0,0.257966,0.615497,2.376,\
		0.0273371,0.619642,2.392));
	cors.push_back(pcl::TMatchingPair(0,0,-0.378274,0.430194,1.947,\
		-0.569029,0.437714,1.915));
	pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> icp;
	CPose3D finalPose;
	icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
	cout<<"ORB: "<<endl;
	finalPose.output(std::cout);
	cout<<endl;

	cors.clear();
	cout<<"SURF: "<<endl;
	cors.push_back(pcl::TMatchingPair(0,0,-0.0532107,0.520158,2.15959,\
		-0.277181,0.529905,2.14));
	cors.push_back(pcl::TMatchingPair(0,0,0.276154,0.605991,2.37968,\
		0.0273371,0.619642,2.392));
	cors.push_back(pcl::TMatchingPair(0,0,-0.367244,0.431617,1.96506,\
		-0.569029,0.437714,1.915));
	icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
	finalPose.output(std::cout);
	cout<<endl;
	for(int t=0;t<10;t++)
	{
		cout<<"The "<<t+1<<" times: "<<endl;
		for(int i=0;i<3;i++)
		{
			cors[i].this_x+=rand_noise();
			cors[i].this_y+=rand_noise();
			cors[i].this_z+=rand_noise();
			cout<<"src:("<<cors[i].other_x<<","<<cors[i].other_y<<","<<cors[i].other_z\
				<<"),dst:("<<cors[i].this_x<<","<<cors[i].this_y<<","<<cors[i].this_z<<")"<<endl;
		}
		icp.leastSquareErrorRigidTransformation6D(cors,finalPose);
		finalPose.output(std::cout);
		cout<<endl;
	}
}

void testORBfeature(string f1, string f2)
{
	//Mat img_1 = imread("D:\\image\\img1.jpg");  
	//Mat img_2 = imread("D:\\image\\img2.jpg");  

	cv::Mat img_1 = cv::imread(f1);
	cv::Mat img_2 = cv::imread(f2);

	if (!img_1.data || !img_2.data)  
	{  
		cout << "error reading images " << endl;  
		return ; 
	}  

	cv::ORB orb;  
	vector<cv::KeyPoint> keyPoints_1, keyPoints_2;  
	cv::Mat descriptors_1, descriptors_2;  

	orb(img_1,cv::Mat(), keyPoints_1, descriptors_1);  
	orb(img_2,cv::Mat(), keyPoints_2, descriptors_2);  

	cv::BruteForceMatcher<cv::HammingLUT> matcher;  
	vector<cv::DMatch> matches;  
	matcher.match(descriptors_1, descriptors_2, matches);  

	double max_dist = 0; double min_dist = 100;  
	//-- Quick calculation of max and min distances between keypoints  
	for( int i = 0; i < descriptors_1.rows; i++ )  
	{   
		double dist = matches[i].distance;  
		if( dist < min_dist ) min_dist = dist;  
		if( dist > max_dist ) max_dist = dist;  
	}  
	printf("-- Max dist : %f \n", max_dist );  
	printf("-- Min dist : %f \n", min_dist );  
	//-- Draw only "good" matches (i.e. whose distance is less than 0.6*max_dist )  
	//-- PS.- radiusMatch can also be used here.  
	std::vector< cv::DMatch > good_matches;  
	for( int i = 0; i < descriptors_1.rows; i++ )  
	{   
		if( matches[i].distance < 0.6*max_dist )  
		{   
			good_matches.push_back( matches[i]);   
		}  
	}  

	cv::Mat img_matches;  
	cv::drawMatches(img_1, keyPoints_1, img_2, keyPoints_2,  
		good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),  
		vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);  
	cv::imshow( "Match", img_matches);  
	cvWaitKey();  
}