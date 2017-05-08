#ifndef MYICP_H
#define MYICP_H

#ifdef BYTE
#undef BYTE
#endif

#ifdef WORD
#undef WORD
#endif

#ifdef UINT
#undef UINT
#endif

#include <iostream>
#include <limits>
#include <fstream>
#include <vector>
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include <boost/shared_ptr.hpp>

#include "CPose3D.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/point_representation.h>

#include <pcl/visualization/pcl_visualizer.h>

//
//using pcl::visualization::PointCloudColorHandlerGenericField;
//using pcl::visualization::PointCloudColorHandlerCustom;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals; 


//// Define a new point representation for < x, y, z, curvature >
//class MyPointRepresentation : public pcl::PointRepresentation <PointNormalT>
//{
//	using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;
//public:
//	MyPointRepresentation ()
//	{
//		// Define the number of dimensions
//		nr_dimensions_ = 4;
//	}
//
//	// Override the copyToFloatArray method to define our feature vector
//	virtual void copyToFloatArray (const PointNormalT &p, float * out) const
//	{
//		// < x, y, z, curvature >
//		out[0] = p.x;
//		out[1] = p.y;
//		out[2] = p.z;
//		out[3] = p.curvature;
//	}
//};

namespace pcl
{

	struct TMatchingPair
	{
		TMatchingPair() :
	this_idx(0), other_idx(0),
		this_x(0),this_y(0),this_z(0),
		other_x(0),other_y(0),other_z(0),
		errorSquareAfterTransformation(0)
	{
	}

	TMatchingPair( unsigned int _this_idx,unsigned int _other_idx, float _this_x, float _this_y,float _this_z, float _other_x,float _other_y,float _other_z ) :
	this_idx(_this_idx), other_idx(_other_idx),
		this_x(_this_x),this_y(_this_y),this_z(_this_z),
		other_x(_other_x),other_y(_other_y),other_z(_other_z),
		errorSquareAfterTransformation(0)
	{
	}

	unsigned int	this_idx;
	unsigned int	other_idx;
	float			this_x,this_y,this_z;
	float			other_x,other_y,other_z;
	float			errorSquareAfterTransformation;
	};

	template <typename PointSource,typename PointTarget>
	class MyICP : public IterativeClosestPoint < PointSource,PointTarget>{
	public:
		using Registration<PointSource,PointTarget>::tree_;
		using Registration<PointSource,PointTarget>::input_;
		using Registration<PointSource,PointTarget>::target_;
		MyICP(){}
		~MyICP(){}
		
		// find intersected points set
		void findIntersectedPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud);

		// ICP Classic method
		void MyicpAlign(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pCloud, boost::shared_ptr<CPose3D> Pose);

		// compute transformation matrix
		bool leastSquareErrorRigidTransformation6D(
			std::vector<pcl::TMatchingPair>	&in_correspondences,
			CPose3D							&out_transformation);

		void cloudRegistration(boost::shared_ptr<CPose3D>& pose, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
		{
			// --------------------------------------------------------------------------
			//  SPECIAL CASE OF HORIZONTAL SCAN: QUICKER IMPLEMENTATION
			// --------------------------------------------------------------------------
			Eigen::Matrix4f	HM;
			pose->getHomogeneousMatrix(HM);

			/*using pcl::transformation instead*/ 
			pcl::transformPointCloud (*cloud, *cloud, HM);
		}

		
		// just for testing leastsqureerror
		void testleastsqureerror();
	};
}

#endif