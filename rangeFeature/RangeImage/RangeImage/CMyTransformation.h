#ifndef CMYTRANSFORMATION_H
#define CMYTRANSFORMATION_H

#include <Eigen/StdVector>
#include <Eigen/Core>
#include <Eigen/Geometry>

class CMyTransformationFromCorrespondences 
{
	public:
		//-----CONSTRUCTOR&DESTRUCTOR-----
		/** Constructor - dimension gives the size of the vectors to work with. */
		CMyTransformationFromCorrespondences ();

		/** Destructor */
		~CMyTransformationFromCorrespondences ();

		//-----METHODS-----
		/** Reset the object to work with a new data set */
		 void reset ();

		/** Get the summed up weight of all added vectors */
		 float getAccumulatedWeight () const { return accumulated_weight_;}

		/** Get the number of added vectors */
		 unsigned int getNoOfSamples () { return no_of_samples_;}

		/** Add a new sample */
		 void add (const Eigen::Vector3f& point, const Eigen::Vector3f& corresponding_point, float weight=1.0);

		/** Calculate the transformation that will best transform the points into their correspondences */
		 Eigen::Affine3f getTransformation ();

		//-----VARIABLES-----

	protected:
		//-----METHODS-----
		//-----VARIABLES-----
		unsigned int no_of_samples_;
		float accumulated_weight_;
		Eigen::Vector3f mean1_, mean2_;
		Eigen::Matrix<float, 3, 3> covariance_;
	};




#endif  // #ifndef PCL_TRANSFORMATION_FROM_CORRESPONDENCES_H


