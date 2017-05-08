#include "preheader.h"
#include "globaldefinition.h"
#include "FusePCD.h"
#include "boost/shared_array.hpp"


void testFuseGT()
{
	CFusePcd tmpFuse("");
	tmpFuse.computeGT();
	cout<<"succeed!"<<endl;
}


void testFusePCD(string file_dir)
{
	CFusePcd tmpFuse(file_dir);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	if(tmpFuse.compute(tmpPC))
		cout<<"succeed!"<<endl;
	else
		cout<<"something wrong!"<<endl;
	//SparsePC(tmpPC,0.04);
	DisplayPT(tmpPC);
	//pcl::io::savePCDFile(string("D:\\MyProjects\\pcl\\lab5.pcd"),*tmpPC);
}

void motionEstimationPCD(string dstf,string srcf)
{
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
}