#ifndef MYMyPCA_H
#define MYMyPCA_H

/** Principal Component analysis (MyPCA) class.\n
*  Principal components are extracted by singular values decomposition on the 
* covariance matrix of the centered input cloud. Available data after pca computation 
* are the mean of the input data, the eigenvalues (in descending order) and 
* corresponding eigenvectors.\n
* Other methods allow projection in the eigenspace, reconstruction from eigenspace and 
*  update of the eigenspace with a new datum (according Matej Artec, Matjaz Jogan and 
* Ales Leonardis: "Incremental MyPCA for On-line Visual Learning and Recognition").
*
* \author Nizar Sallem
* \ingroup common
*/

#include <pcl/pcl_base.h>
#include <pcl/pcl_macros.h>
namespace pcl{
	template <typename PointT>
	class MyPCA : public pcl::PCLBase <PointT>
	{
	public:
		typedef pcl::PCLBase <PointT> Base;
		typedef typename Base::PointCloud PointCloud;
		typedef typename Base::PointCloudPtr PointCloudPtr;
		typedef typename Base::PointCloudConstPtr PointCloudConstPtr;
		typedef typename Base::PointIndicesPtr PointIndicesPtr;
		typedef typename Base::PointIndicesConstPtr PointIndicesConstPtr;

		using Base::input_;
		using Base::indices_;
		using Base::initCompute;
		using Base::setInputCloud;

		/** Updating method flag */
		enum FLAG 
		{
			/** keep the new basis vectors if possible */
			increase, 
			/** preserve subspace dimension */
			preserve
		};

		/** \brief Default Constructor
		* \param basis_only flag to compute only the MyPCA basis
		*/
		MyPCA (bool basis_only = false)
			: Base ()
			, compute_done_ (false)
			, basis_only_ (basis_only) 
		{}


		/** Copy Constructor
		* \param[in] pca MyPCA object
		*/
		MyPCA (MyPCA const & pca) 
			: Base (pca)
			, eigenvectors_ (pca.eigenvectors_)
			, coefficients_ (pca.coefficients_)
			, mean_ (pca.mean_)
			, eigenvalues_  (pca.eigenvalues_)
		{}

		/** Assignment operator
		* \param pca MyPCA object
		*/
		inline MyPCA& operator= (MyPCA const & pca) 
		{
			eigenvectors_ = pca.eigenvectors;
			coefficients_ = pca.coefficients;
			eigenvalues_  = pca.eigenvalues;
			mean_         = pca.mean;
			return (*this);
		}

		/** \brief Provide a pointer to the input dataset
		* \param cloud the const boost shared pointer to a PointCloud message
		*/
		inline void 
			setInputCloud (const PointCloudConstPtr &cloud) 
		{ 
			Base::setInputCloud (cloud);
			compute_done_ = false;
		}

		/** \brief Mean accessor
		* \throw InitFailedException
		*/
		inline Eigen::Vector4f& 
			getMean () 
		{
			if (!compute_done_)
				initCompute ();
			if (!compute_done_)
				PCL_THROW_EXCEPTION (InitFailedException, 
				"[pcl::MyPCA::getMean] MyPCA initCompute failed");
			return (mean_);
		}

		/** Eigen Vectors accessor
		* \throw InitFailedException
		*/
		inline Eigen::Matrix3f& 
			getEigenVectors () 
		{
			if (!compute_done_)
				initCompute ();
			if (!compute_done_)
				PCL_THROW_EXCEPTION (InitFailedException, 
				"[pcl::MyPCA::getEigenVectors] MyPCA initCompute failed");
			return (eigenvectors_);
		}

		/** Eigen Values accessor
		* \throw InitFailedException
		*/
		inline Eigen::Vector3f& 
			getEigenValues ()
		{
			if (!compute_done_)
				initCompute ();
			if (!compute_done_){}
				/*
				PCL_THROW_EXCEPTION (InitFailedException, 
								"[pcl::MyPCA::getEigenVectors] MyPCA getEigenValues failed");*/
				
			return (eigenvalues_);
		}

		/** Coefficients accessor
		* \throw InitFailedException
		*/
		inline Eigen::MatrixXf& 
			getCoefficients () 
		{
			if (!compute_done_)
				initCompute ();
			if (!compute_done_)
				PCL_THROW_EXCEPTION (InitFailedException, 
				"[pcl::MyPCA::getEigenVectors] MyPCA getCoefficients failed");
			return (coefficients_);
		}

		/** update MyPCA with a new point
		* \param[in] input input point 
		* \param[in] flag update flag
		* \throw InitFailedException
		*/
		inline void 
			update (const PointT& input, FLAG flag = preserve);

		/** Project point on the eigenspace.
		* \param[in] input point from original dataset
		* \param[out] projection the point in eigen vectors space
		* \throw InitFailedException
		*/
		inline void 
			project (const PointT& input, PointT& projection);

		/** Project cloud on the eigenspace.
		* \param[in] input cloud from original dataset
		* \param[out] projection the cloud in eigen vectors space
		* \throw InitFailedException
		*/
		inline void
			project (const PointCloud& input, PointCloud& projection);

		/** Reconstruct point from its projection
		* \param[in] projection point from eigenvector space
		* \param[out] input reconstructed point
		* \throw InitFailedException
		*/
		inline void 
			reconstruct (const PointT& projection, PointT& input);

		/** Reconstruct cloud from its projection
		* \param[in] projection cloud from eigenvector space
		* \param[out] input reconstructed cloud
		* \throw InitFailedException
		*/
		inline void
			reconstruct (const PointCloud& projection, PointCloud& input);

	private:
		inline bool
			initCompute ();

		bool compute_done_;
		bool basis_only_;
		Eigen::Matrix3f eigenvectors_;
		Eigen::MatrixXf coefficients_;
		Eigen::Vector4f mean_;
		Eigen::Vector3f eigenvalues_;
	}; // class MyPCA
}

#include "MyPCA.hpp"



#endif