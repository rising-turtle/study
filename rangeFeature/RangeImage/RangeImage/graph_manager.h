#pragma once

//
#include "node.h"
#include <aislib/graph_optimizer_hogman/graph_optimizer3d_hchol.h>
#include <aislib/graph/loadEdges3d.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "preheader.h"
#include <memory> //for auto_ptr
//#include "glviewer.h"
#include "globaldefinition.h"
#include "CPose3D.h"
//#define ROSCONSOLE_SEVERITY_INFO

class COrbNode;

namespace AIS = AISNavigation;


extern Transformation3 eigen2Hogman(const Eigen::Matrix4f& eigen_mat);

//!Computes a globally optimal trajectory from transformations between Node-pairs
//class GraphManager{
//	friend class CVisualSlam;
//    /// Start over with new graph
//    void reset();
//    ///iterate over all Nodes, sending their transform and pointcloud
//    //void sendAllClouds();
//    ///Call saveIndividualCloudsToFile, if possible as background thread
//    //void saveIndividualClouds(QString file_basename);
//    ///Call saveAllCloudsToFile, if possible as background thread
//    //void saveAllClouds(QString filename);
//    ///Throw the last node out, reoptimize
//	void eraseNode(Node* node);
//	bool deleteOutRangeFrames(double x, double y, double z, double radius);
//public:
//    void deleteLastFrame(); 
//    void setMaxDepth(float max_depth);
//
//    public:
//    GraphManager(/*GLViewer* glviewer*/);
//    ~GraphManager();
//
//    /// Add new node to the graph.
//    /// Node will be included, if a valid transformation to one of the former nodes
//    /// can be found. If appropriate, the graph is optimized
//    /// graphmanager owns newNode after this call. Do no delete the object
//    bool addNode(Node* newNode); 
//
//    ///Flag to indicate that the graph is globally corrected after the addNode call.
//    ///However, currently optimization is done in every call anyhow
//    bool freshlyOptimized_;
//    //GLViewer glviewer_;
//
//    std::map<int, Node* > graph_;
//	double latest_pose[6];
//
//    void flannNeighbours();
//
//    float Max_Depth;
//    //void setMaxDepth(float max_depth);
//
//	// for saving matched images
//	MatchingResult lastmr;
//
//	// testOptimizer
//	void testOptimizer();
//protected:
//
//    std::vector < cv::DMatch > last_inlier_matches_;
//    std::vector < cv::DMatch > last_matches_;
//    /// The parameter max_targets determines how many potential edges are wanted
//    /// max_targets < 0: No limit
//    /// max_targets = 0: Compare to first frame only
//    /// max_targets = 1: Compare to previous frame only
//    /// max_targets > 1: Select intelligently
//    std::vector<int> getPotentialEdgeTargets(const Node* new_node, int max_targets);
//    
//    std::vector<int> getPotentialEdgeTargetsFeatures(const Node* new_node, int max_targets);
//    
//    void optimizeGraph(bool online=true);
//    void initializeHogman();
//    bool addEdgeToHogman(AIS::LoadedEdge3D edge, bool good_edge);
//
//    void resetGraph();
//
//    void mergeAllClouds(pointcloud_type & merge);
//    
//    AIS::GraphOptimizer3D* optimizer_;
//
//    //tf::TransformBroadcaster br_;
//    //tf::Transform kinect_transform_; ///<transformation of the last frame to the first frame (assuming the first one is fixed)
//    //Eigen::Matrix4f latest_transform_;///<same as kinect_transform_ as Eigen
//
//
//    // true if translation > 10cm or largest euler-angle>5 deg
//    // used to decide if the camera has moved far enough to generate a new nodes
//    bool isBigTrafo(const Eigen::Matrix4f& t);
//    bool isBigTrafo(const Transformation3& t);
//
//    /// get euler angles from 4x4 homogenous
//    void static mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw);
//    /// get translation-distance from 4x4 homogenous
//    void static mat2dist(const Eigen::Matrix4f& t, double &dist);
//    void mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist);
//
//    bool reset_request_;
//    std::clock_t last_batch_update_;
//    unsigned int marker_id;
//    int last_matching_node_;
//    bool batch_processing_runs_;
//
//};
class GraphManager{
	/// Start over with new graph
	void reset();
	///iterate over all Nodes, sending their transform and pointcloud
	//void sendAllClouds();
	///Call saveIndividualCloudsToFile, if possible as background thread
	//void saveIndividualClouds(QString file_basename);
	///Call saveAllCloudsToFile, if possible as background thread
	//void saveAllClouds(QString filename);
	///Throw the last node out, reoptimize
	void eraseNode(Node* node);
	bool deleteOutRangeFrames(double x, double y, double z, double radius);
public:
	void deleteLastFrame(); 
	void setMaxDepth(float max_depth);

public:
	GraphManager(/*GLViewer* glviewer*/);
	~GraphManager();

	/// Add new node to the graph.
	/// Node will be included, if a valid transformation to one of the former nodes
	/// can be found. If appropriate, the graph is optimized
	/// graphmanager owns newNode after this call. Do no delete the object
	bool addNode(Node* newNode); 
	bool addNode2(Node* newNode); 
	
	// 尝试加入ORB-Node进行后端优化
	bool addORBNode(COrbNode* new_node);

	///Flag to indicate that the graph is globally corrected after the addNode call.
	///However, currently optimization is done in every call anyhow
	bool freshlyOptimized_;
	//GLViewer glviewer_;

	std::map<int, COrbNode*> orb_graph_;
	std::map<int, Node* > graph_;
	double latest_pose[6];

	void flannNeighbours();

	float Max_Depth;
	//void setMaxDepth(float max_depth);

	bool matched;
	// for saving matched images
	MatchingResult lastmr;

	bool usingIMU;
	CPose3D last_pose;
	CPose3D curr_pose;

public: 
	// 输出所包含的点云信息到一个点云中
	bool FuseOutPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_outPC);
	bool FuseOutPC_ORB(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_outPC);
	// 利用矢量化的结果显示点云
	bool FuseOutPCVector(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_outPC);
	
	// 输出路径信息
	void outputTrajectory(ostream& out);

	// 稀疏稠密点云，并且删掉其中_nan的无效点
	void FilterDensePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_densePC);

	//protected:
public:
	std::vector < cv::DMatch > last_inlier_matches_;
	std::vector < cv::DMatch > last_matches_;
	/// The parameter max_targets determines how many potential edges are wanted
	/// max_targets < 0: No limit
	/// max_targets = 0: Compare to first frame only
	/// max_targets = 1: Compare to previous frame only
	/// max_targets > 1: Select intelligently
	std::vector<int> getPotentialEdgeTargets(const Node* new_node, int max_targets);

	std::vector<int> getPotentialEdgeTargetsFeatures(const Node* new_node, int max_targets);
	// 重载到ORBNode上来
	std::vector<int> getPotentialEdgeTargets(const COrbNode* new_node, int max_targets);

	void optimizeGraph(bool online=true);
	void optimizeGraph2(bool online=true);
	void initializeHogman();
	bool addEdgeToHogman(AIS::LoadedEdge3D edge, bool good_edge);
	bool addEdgeToHogman2(AIS::LoadedEdge3D edge, bool good_edge);

	void resetGraph();

	//void mergeAllClouds(pointcloud_type & merge);

	AIS::GraphOptimizer3D* optimizer_;
	AIS::GraphOptimizer3D* bg_optimizer_;

	//tf::TransformBroadcaster br_;
	//tf::Transform kinect_transform_; ///<transformation of the last frame to the first frame (assuming the first one is fixed)
	//Eigen::Matrix4f latest_transform_;///<same as kinect_transform_ as Eigen


	// true if translation > 10cm or largest euler-angle>5 deg
	// used to decide if the camera has moved far enough to generate a new nodes
	bool isBigTrafo(const Eigen::Matrix4f& t);
	bool isBigTrafo(const Transformation3& t);

	bool isNoiseTrafo(const Eigen::Matrix4f& t);
	bool isNoiseTrafo(const Transformation3& t);

	/// get euler angles from 4x4 homogenous
	void static mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw);
	/// get translation-distance from 4x4 homogenous
	void static mat2dist(const Eigen::Matrix4f& t, double &dist);
	void mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist);
	void GraphManager::testOptimizer();
	bool reset_request_;
	std::clock_t last_batch_update_;
	unsigned int marker_id;
	int last_matching_node_;
	bool batch_processing_runs_;

};
//void transformAndAppendPointCloud (const pointcloud_type &cloud_in, pointcloud_type &cloud_to_append_to,
//                                   const tf::Transform transformation, float Max_Depth);