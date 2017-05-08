#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include <Eigen/StdVector>
//#include <image_geometry/pinhole_camera_model.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>
#include "globaldefinition.h"

#include "matching_result.h" 
#include "CPose3D.h"
#include "MyICP.h"

// Search structure for descriptormatching
typedef cv::flann::Index cv_flannIndex;

//!Holds the data for one graph node and provides functionality to compute relative transformations to other Nodes.
class Node {
public:
	///Visual must be CV_8UC1, depth CV_32FC1, 
	///id must correspond to the hogman vertex id
	///detection_mask must be CV_8UC1 with non-zero 
	///at potential keypoint locations
	Node(const cv::Mat& visual,
			cv::Ptr<cv::FeatureDetector> detector,
			cv::Ptr<cv::DescriptorExtractor> extractor,
			cv::Ptr<cv::DescriptorMatcher> matcher, // deprecated!
			const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > point_cloud,
			const cv::Mat& detection_mask = cv::Mat());
	//default constructor. TODO: still needed?
	Node(){}
	///Delete the flannIndex if built
	~Node();


	///Compare the features of two nodes and compute the transformation
	MatchingResult matchNodePair(const Node* older_node);

	///Compare the features of adjacent node current and last node
	MatchingResult matchLastNode(const Node* older_node, CPose3D& last_pose, CPose3D& curr_pose);

	// 获取特征点的3D坐标
	void getFeatureLoc(vector<int> indices,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_out);


	///Compute the relative transformation between the nodes
	///Do either max_ransac_iterations or half of it, 
	///Iterations with more than half of the initial_matches 
	///inlying, count twice. Iterations with more than 80% of 
	///the initial_matches inlying, count threefold
	bool getRelativeTransformationTo(const Node* target_node, 
			std::vector<cv::DMatch>* initial_matches,
			Eigen::Matrix4f& resulting_transformation, 
			float& rmse,
			std::vector<cv::DMatch>& matches,//for visualization?
					unsigned int max_ransac_iterations = 1000) const;

	void buildFlannIndex();
	int findPairsFlann(const Node* other, vector<cv::DMatch>* matches) const;



	//PointCloud pc;
	///pcl::PointCloud<pcl::PointXYZRGB> centrally defines what the pc is templated on
	//pcl::PointCloud<pcl::PointXYZRGB> pc_col;
	
	// node 所观测得到的点云信息
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pPC;

	cv::Mat feature_descriptors_;         ///<descriptor definitions
	std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > feature_locations_3d_;  ///<backprojected 3d descriptor locations relative to cam position in homogeneous coordinates (last dimension is 1.0)
	std::vector<cv::KeyPoint> feature_locations_2d_; ///<Where in the image are the descriptors
	unsigned int id_; ///must correspond to the hogman vertex id

protected:


	cv_flannIndex* flannIndex;
//	image_geometry::PinholeCameraModel cam_model_;  
	cv::Ptr<cv::DescriptorMatcher> matcher_;

	void projectTo3D(std::vector<cv::KeyPoint>& feature_locations_2d,
			std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& feature_locations_3d,
			const pcl::PointCloud<pcl::PointXYZRGB>& point_cloud);
	/*
    ///Compare the features of two nodes and compute the transformation
    void processNodePair(const Node* older_node, 
                         std::vector<cv::DMatch>& inliers,std::vector<cv::DMatch>& outliers,
                         AISNavigation::LoadedEdge3D& edge_out,
                         Eigen::Matrix4f& ransac_transform, Eigen::Matrix4f& final_trafo) const;
	 */
	// helper for ransac
	// check for distances only if max_dist_cm > 0
	template<class InputIterator>
	Eigen::Matrix4f getTransformFromMatches(const Node* other_node, 
			InputIterator iter_begin,
			InputIterator iter_end,
			bool* valid = NULL, 
			float max_dist_m = -1
	) const;
	//std::vector<cv::DMatch> const* matches,
	//pcl::TransformationFromCorrespondences& tfc);


	///Get the norm of the translational part of an affine matrix (Helper for isBigTrafo)
	void mat2dist(const Eigen::Matrix4f& t, double &dist){
		dist = sqrt(t(0,3)*t(0,3)+t(1,3)*t(1,3)+t(2,3)*t(2,3));
	}
	///Get euler angles from affine matrix (helper for isBigTrafo)
	void mat2RPY(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw) ;

	void mat2components(const Eigen::Matrix4f& t, double& roll, double& pitch, double& yaw, double& dist);

	// helper for ransac
	void computeInliersAndError(const std::vector<cv::DMatch>& initial_matches,
			const Eigen::Matrix4f& transformation,
			const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& origins,
			const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& targets,
			std::vector<cv::DMatch>& new_inliers, //output var
			double& mean_error, vector<double>& errors,
			double squaredMaxInlierDistInM = 0.0009) const; //output var;

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
