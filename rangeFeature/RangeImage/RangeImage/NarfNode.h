#ifndef NORFNODE_H
#define NORFNODE_H

#include "preheader.h"

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "matching_result.h" 
#include "MyICP.h"
#include "Narf_kdtree_flann.h"

#define CLOSEDIS 0.004
//#define CLOSEDIS 1e-3

class CNarfNode{
public:
	CNarfNode();
	CNarfNode(string filename);
	~CNarfNode();
	bool Init(string filename);
public:
	void FindPairsFlann(CNarfNode* pOldNode,vector<cv::DMatch>* out_matches,bool useNarf=true,int _k=2);
	void FindPairsFlann2(CNarfNode* pOldNode,vector<cv::DMatch>* out_matches,int num);
	void DeleteMultiMatches(vector<cv::DMatch>* out_matches);
	void FindMultiPairsFlann(CNarfNode* pOldNode,vector<cv::DMatch>* out_matches,int num);
	MatchingResult MatchNodePair(CNarfNode* older_node);
	void CalPFHwithKeyPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &points, pcl::PointCloud<pcl::Normal>::Ptr &normals, 
		pcl::PointCloud<pcl::PointXYZ>::Ptr &keypoints, pcl::PointCloud<pcl::PFHSignature125>::Ptr& descriptors_out);
	void Downsample (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float leaf_size,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr &downsampled_out);
	void ComputeSurfaceNormals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float normal_radius,
		pcl::PointCloud<pcl::Normal>::Ptr &normals_out);

public:
	boost::shared_ptr<pcl::RangeImage > m_pRangeImage;
	pcl::PointCloud<pcl::PointWithViewpoint>::Ptr m_pFarRanges;
	pcl::NarfKeypoint m_narf_keypoint_detector;
	Eigen::Affine3f m_scene_sensor_pose;

	// id in the node-graph
	int m_id;
	// whether this node is valid
	bool m_ValidNode;
	// Narf key points
	vector<int> m_keypoint_indices;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pPC;
	pcl::PointCloud<pcl::Narf36>::Ptr m_pNarfDest;
	boost::shared_ptr<pcl::NarfKdTreeFLANN<pcl::Narf36> > m_pNarfKDFlann;
	// PFH key points
	pcl::PointCloud<pcl::PointXYZ>::Ptr m_pfhKeyPoints;
	pcl::PointCloud<pcl::Normal>::Ptr m_pNormals;
	pcl::PointCloud<pcl::PFHSignature125>::Ptr m_pPFHDest;
	boost::shared_ptr<pcl::KdTreeFLANN<pcl::PFHSignature125> > m_pPFHKDFlann;

public:
	static float narfnode_angular_resolution;// = 0.5f;
	static float narfnode_support_size;// = 0.2f;
	static pcl::RangeImage::CoordinateFrame narfnode_coordinate_frame;// = pcl::RangeImage::CAMERA_FRAME;
	static bool narfnode_setUnseenToMaxRange;// = false;
	static float narfnode_noise_level;// = 0.0;
	static float narfnode_min_range;// = 0.0f;
	static int narfnode_border_size;// = 1;
	static bool narfnode_rotation_invariant;
	// PFH search radius
	static float pfh_feature_radius;
	static float pfh_voxel_grid_leaf_size;
	static float pfh_normal_radius;

public:
	///Compute the relative transformation between the nodes
	///Do either max_ransac_iterations or half of it, 
	///Iterations with more than half of the initial_matches 
	///inlying, count twice. Iterations with more than 80% of 
	///the initial_matches inlying, count threefold
	bool getRelativeTransformationTo(CNarfNode* target_node, 
		std::vector<cv::DMatch>* initial_matches,
		Eigen::Matrix4f& resulting_transformation, 
		float& rmse,
		std::vector<cv::DMatch>& matches,//for visualization?
		unsigned int max_ransac_iterations = 1000);
	void computeInliersAndError(std::vector<cv::DMatch>& matches,
		Eigen::Matrix4f& transformation,
		vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ> >& origins,
		vector<pcl::PointXYZ,Eigen::aligned_allocator<pcl::PointXYZ> >& earlier,
		std::vector<cv::DMatch>& inliers, //output var
		double& mean_error,
		vector<double>& errors,
		double squaredMaxInlierDistInM=0.0009);
	template<class InputIterator>
	Eigen::Matrix4f getTransformFromMatches(const CNarfNode* earlier_node,
		InputIterator iter_begin,
		InputIterator iter_end,
		bool* valid=NULL, 
		float max_dist_m=-1);
public:
	
	// mapping Narf KeyPoint with KeyPoints
	int findNarfKeyPoint(pcl::Narf36& );
	
	// find (0,0,0) points, mark this as nan
	template<typename PT>
	void FindZeroPoints(boost::shared_ptr<pcl::PointCloud<PT> >& m_pt)
	{
		for(size_t i=0;i<m_pt->points.size();i++)
		{
			PT& pt=m_pt->points[i];
			if(pt.x==0 && pt.y ==0 && pt.z ==0)
			{
				pt.x=pt.y=pt.z=numeric_limits<double>::infinity();
				/*if(!(pcl_isfinite (pt.x) && pcl_isfinite (pt.y) && pcl_isfinite (pt.z)))
					cout<<"succeed to mark this point infinite!"<<endl;*/
			}
			
		}
	}

	template<typename PT>
	bool IsSamePoint(const PT& p1,const PT& p2)
	{
		if((sqr(p1.x-p2.x)+sqr(p1.y-p2.y)+sqr(p1.z-p2.z))<CLOSEDIS)
			return true;
		return false;
	}
	template<typename T>
	T sqr(T x){return (x*x);}

};


#endif