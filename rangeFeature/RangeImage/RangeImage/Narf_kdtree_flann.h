#ifndef NARFKDTREEFLANN_H
#define NARFKDTREEFLANN_H

#include "Narf_point_representation.h"
#include "pcl/kdtree/kdtree_flann.h"

namespace pcl{
	template<typename PointT>
	class NarfKdTreeFLANN : public pcl::KdTreeFLANN<PointT>
	{

	}; 
}


#endif