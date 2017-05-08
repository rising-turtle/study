#ifndef NARFPOINTREPRESENTATION_H
#define NARFPOINTREPRESENTATION_H

#include "preheader.h"
#include "pcl/point_representation.h"

namespace pcl{
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	template <>
	class DefaultPointRepresentation <Narf36> : public  PointRepresentation <Narf36>
	{
	public:
		DefaultPointRepresentation ()
		{
			nr_dimensions_ = 36;
		}

		virtual void 
			copyToFloatArray (const Narf36 &p, float * out) const
		{
			for (int i = 0; i < nr_dimensions_; ++i)
				out[i] = p.descriptor[i];
		}
	};
}

#endif