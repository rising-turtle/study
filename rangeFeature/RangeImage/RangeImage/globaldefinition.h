#ifndef GLOBALDEFINITION_H
#define GLOBALDEFINITION_H

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <Eigen/StdVector>
#include <Eigen/Core>
#include <aislib/math/transformation.h>
#include "opencv2/core/core.hpp"  
#include "opencv2/highgui/highgui.hpp" 

// PCL Examples
extern int range_image_border_extraction_main(string file_name);
extern int range_image_creation_main(int argc,char** argv);
extern int extract_narf_from_range_image_main (int argc, char** argv);
extern int normal_distributions_transform_main (int argc, char** argv);

// Hogman function
extern Transformation3 eigen2Hogman(const Eigen::Matrix4f& eigen_mat);

// test NarfNode
extern void testRoompcd();
extern void testNarfNode();
extern void testNarfMatchFeatures();
extern void testDisNarfNode();

// for RGBD SLAM
///This influences speed dramatically
extern  int global_adjuster_max_keypoints;
extern  int global_adjuster_min_keypoints;
extern  int global_fast_adjuster_max_iterations;
extern  int global_surf_adjuster_max_iterations;

extern  float global_min_translation_meter;
extern  float global_min_rotation_degree; 

///(lower=faster, higher=better loop closing)
extern  unsigned int global_connectivity;
extern  unsigned int global_potential_nodes;

// spatial limitation for Nodes
extern  double global_graph_radius;
extern  int global_graph_size;
extern  int global_bg_graph_threshold;
extern  double global_max_translation_meter;
extern unsigned int global_min_inliers;

#define _LOG 1
// for log
extern int gl_pf_fid;
extern int gl_pf_gs;
extern double gl_pf_fe;
extern double gl_pf_fm;
extern double gl_pf_id;
extern double gl_pf_me;
extern double gl_pf_slam;
extern double gl_pf_start;
extern double gl_pf_op;

typedef enum _FEATURETYPE{PFH,NARF,PFHNARF}FeatureType;
void showMatchedFeatures(string file_dir,int num1,int num2,FeatureType feature_type,int _k=2);
void testConferenceRoomSet();

// for test conference room 
extern void testConferenceRoom(int num1,int num2);

// for pose estimation
extern void motionEstimationPCD(string dir);
extern void motionEstimationPCD(string dstf,string srcf);
extern void getImagesandDepthMetaData(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud,
									  unsigned char* rgbbuf,unsigned char* depthbuf);

// for segmentation
extern void testVoxelSparse(string file_name);
extern void testSparseNormal(string file_name);
extern void testNormalSegment(string file_name);
extern void testHueSegment(string file_name);
extern void testHueNormalSegment(string file_name);
extern void testPlaneInfo(string file_name);
extern void testFindPlanes(string file_name);
extern void testFrameSeg(string file_dir);

// for edge contour extraction
extern void testEdgeContourExtraction(string file_name);

// for analysis surfaces
extern void testPlaneAnalysis(string file_name);

// for triangulation 
extern int fast_greedy_triangulation (string file_name);
extern int showTriangulation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, pcl::PointCloud<pcl::Normal>::Ptr& normals);
extern void testTriangulation(string file_name);

// for all process: segment->planes->blend->padding->contour->triangulation
extern void testTransfer(string file_name);

extern void testTC(string file_name);

// test 3DS Writer
extern void test3ds(string file_in, string file_out);
extern void testLoad3ds(string file_in);

// test from .pcd to .3ds
extern void testPC23DS(string file_in, string file_out);

// test Fuse *pcds into single pcd
extern void testFusePCD(string file_dir);

// test ORB features
extern void testORBMatch(string dstf, string srcf);
extern void testORBPCD(string dstf, string srcf);
extern void testORBfeature(string f1, string f2);
extern void testORBMatchShow(string dstf, string srcf);
extern void testORBMatch(string file_dir);
extern void testSURFMatchShow(string dstf, string srcf);
extern void testMatchMatrix(string dstf, string srcf);
extern void testFixMatch(string dstf,string srcf);
extern void testMatrix();

// 测试所抓取的image 与 depth是否匹配
extern void showBitNodes();

// 测试对image中的tag提取是否正确
extern void testTagFinder(string file_name);
extern int testMorphoFeature(string file_name);
extern int testEdgeExtract(string file_name);
extern void testTagFinderAll(string file_dir);
extern void testTagLocation();
extern void calPCfromImageAndDepth(cv::Mat& image, cv::Mat& depth, \
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr& outPC);

void testVariableRefine();
// 利用April提供的位置索引找到对应的
void testAprilSpecified();

// 测试面向量转换与面融合
void testFusePlanes(string file_name);

// 提取点云中的图像与深度信息
void getImageDepth(string file_name);
// 为了实验能否提高面的融合度
void testExperiment();

// 测试同步获取点云
void testGroundTruth();
void testFuseGT();

//  random noise 
extern double rand_noise();
#include "globalimpl.hpp"


#endif