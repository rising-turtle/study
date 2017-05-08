#include "globaldefinition.h"
#include "preheader.h"
#include "CPose3D.h"
namespace{
	//string file_dir("D:\\myproj\\pcl\\conference_room\\conference_room");
	string file_dir("D:\\myproj\\pcl\\RangeImage\\Debug\\");
	int _k=7;
}
void main(int argc,char** argv)
{
	//testFuseGT();

	//testGroundTruth();
	//testExperiment();

	string edgefile("D:\\myproj\\images\\boldt.jpg");
	string morfile("D:\\myproj\\images\\building.jpg");
	string file_tag("D:\\myproj\\KinectDeviation\\cornerloc\\exper2\\i2.png");
	string tag_dir("D:\\myproj\\KinectDeviation\\cornerloc\\exper2\\");
	//testAprilSpecified();
	//testVariableRefine();
	//testTagLocation();
	//testTagFinderAll(tag_dir);
	//testEdgeExtract(file_tag);
	//testTagFinder(file_tag);
	//testMorphoFeature(file_tag);
	//showBitNodes();

	string file_name("D:\\myproj\\pcl\\pcds\\10.pcd");
	//string file_name("D:\\myproj\\pcl\\lab3.pcd");

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile(file_name,*mPC);
	cout<<"load successful"<<endl;
	DisplayPT(mPC);
	cout<<"succeed!"<<endl;

	string file_3ds_name("C:\\Users\\Iot\\Desktop\\mat\\lab.3ds");
	//testPC23DS(file_name,file_3ds_name);
	testPlaneAnalysis(file_name);
	//testSparseNormal(file_name);
	//testHueSegment(file_name);
	//testHueNormalSegment(file_name);
	//testPlaneInfo(file_name);
	string file_name1("D:\\myproj\\pcl\\pcds\\20.pcd");
	//testFindPlanes(file_name1);
	//getImageDepth(file_name1);

	string file_dir("D:\\myproj\\pcl\\pcds\\");
	//testFrameSeg(file_dir);

	//testFusePlanes(file_name1);
	//testFusePCD(file_dir);
	//testORBMatch(file_dir);

	//testTransfer(file_name);
	//testTriangulation(file_name);
	//fast_greedy_triangulation(file_name);
	//testTC(file_name);

	//testEdgeContourExtraction(file_name);
	//range_image_border_extraction_main(file_name);
	//range_image_creation_main(argc,argv);
	//extract_narf_from_range_image_main(argc,argv);
	//testRoompcd();
	//testDisNarfNode();
	//testNarfMatchFeatures();
	//testConferenceRoom(0,1);
	//showMatchedFeatures(file_dir,1,2,FeatureType::PFHNARF,_k);
	//testConferenceRoomSet();
	//motionEstimationPCD("D:\\myproj\\pcl\\RangeImage\\Debug");
	string dstf("D:\\myproj\\pcl\\pcds\\3.pcd");
	string srcf("D:\\myproj\\pcl\\pcds\\4.pcd");
	//testMatrix();
	//testFixMatch(dstf,srcf);
	//testMatchMatrix(dstf,srcf);
	//testORBMatch(dstf,srcf);
	//testORBPCD(dstf,srcf);
	//testORBMatchShow(dstf,srcf);
	//testSURFMatchShow(dstf,srcf);
	string f1("D:\\myproj\\ORBFeature\\images\\frame1.bmp");
	string f2("D:\\myproj\\ORBFeature\\images\\frame2.bmp");
	//testORBfeature(f1,f2);
	//motionEstimationPCD(dstf,srcf);
	//testVoxelSparse("D:\\myproj\\pcl\\pcds\\10.pcd");
	//testNormalSegment(file_name.c_str());
	
	string file_in("D:\\myproj\\pcl\\3ds\\house.3DS");
	string file_out("D:\\myproj\\pcl\\3ds\\my3ds\\house.3DS");
	//test3ds(file_in,file_out);
	//getchar();
	//string file_tmp("D:\\myproj\\pcl\\3ds\\my3ds\\tmp.3DS");
	//testLoad3ds(file_in);
	//testLoad3ds(file_3ds_name);
	getchar();
	return ;
}