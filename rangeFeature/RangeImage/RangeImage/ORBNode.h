#ifndef ORB_NODE_H
#define ORB_NODE_H

#include "preheader.h"
#include "globaldefinition.h"

#include "opencv2/core/core.hpp"  
#include "opencv2/features2d/features2d.hpp"  
#include "opencv2/highgui/highgui.hpp" 

#include <Eigen/Core>
#include <Eigen/StdVector>
#include "matching_result.h" 
#include "MyICP.h"

// Search structure for descriptormatching
typedef cv::flann::Index cv_flannIndex;

class COrbNode
{
public:
	COrbNode(const cv::Mat& visual,pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc);
	~COrbNode();
public:
	///Compare the features of two nodes and compute the transformation
	MatchingResult matchNodePair(const COrbNode* older_node,bool Isadj=true);
	void findPairsBruteForce(const COrbNode* older_node,std::vector<cv::DMatch>& matches); // 找到匹配的feature pairs
	void findPairsBruteForceGPU(const COrbNode* older_node,std::vector<cv::DMatch>& matches); //

	// 获取指定的特征点的3D坐标
	void getFeatureLoc(vector<int> indices,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_out);
	// 把相距小于m_sMinFeatureDis的特征点对都删掉
	void deleteSamePairs(vector<cv::DMatch>& out_match) const;
public:
	// 特征点之间最小的距离
	static double m_sMinFeatureDis;

	//pcl::PointCloud<pcl::PointXYZRGB> pc_col;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pPC;
	// ORB features
	cv::ORB* m_porb;  
	vector<cv::KeyPoint> m_keyPoints;  // 2D feature points
	cv::Mat m_descriptors;			   // ORB descriptor
	vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > m_feature_locations_3d_;  // 3D 坐标
	unsigned int m_id;			   // Node id
	cv_flannIndex* m_flannIndex;     // Feature KDTree 不知道有没有用
public:
	void buildflannIndex();
	int findPairsFlann(const COrbNode* other, vector<cv::DMatch>* matches) const;
	// backprojected 3d descriptor locations relative to cam position in homogeneous coordinates (last dimension is 1.0)
	void projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,cv::Mat& feature_descriptor,
		std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
		const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud);
	///Compute the relative transformation between the nodes
	///Do either max_ransac_iterations or half of it, 
	///Iterations with more than half of the initial_matches 
	///inlying, count twice. Iterations with more than 80% of 
	///the initial_matches inlying, count threefold
	bool getRelativeTransformationTo(const COrbNode* target_node, 
		std::vector<cv::DMatch>* initial_matches,
		Eigen::Matrix4f& resulting_transformation, 
		float& rmse,
		std::vector<cv::DMatch>& matches,//for visualization?
		bool Isadj=true,
		unsigned int max_ransac_iterations = 1000) const;
	template<class InputIterator>
	Eigen::Matrix4f getTransformFromMatches(const COrbNode* other_node, 
		InputIterator iter_begin,
		InputIterator iter_end,
		bool* valid = NULL, 
		float max_dist_m = -1
		) const;
	// helper for ransac
	void computeInliersAndError(const std::vector<cv::DMatch>& initial_matches,
		const Eigen::Matrix4f& transformation,
		const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
		const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& targets,
		std::vector<cv::DMatch>& new_inliers, //output var
		double& mean_error, vector<double>& errors,
		double squaredMaxInlierDistInM = 0.0009) const; //output var;
private:
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif