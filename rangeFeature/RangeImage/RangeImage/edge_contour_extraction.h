 /*
  * Software License Agreement (BSD License)
  *
  *  Point Cloud Library (PCL) - www.pointclouds.org
  *  Copyright (c) 2010-2012, Willow Garage, Inc.
  *
  *  All rights reserved.
  *
  *  Redistribution and use in source and binary forms, with or without
  *  modification, are permitted provided that the following conditions
  *  are met:
  *
  *   * Redistributions of source code must retain the above copyright
  *     notice, this list of conditions and the following disclaimer.
  *   * Redistributions in binary form must reproduce the above
  *     copyright notice, this list of conditions and the following
  *     disclaimer in the documentation and/or other materials provided
  *     with the distribution.
  *   * Neither the name of Willow Garage, Inc. nor the names of its
  *     contributors may be used to endorse or promote products derived
  *     from this software without specific prior written permission.
  *
  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  *  POSSIBILITY OF SUCH DAMAGE.
  *
  */

#ifndef PCL_FILTERS_EDGE_CONTOUR_EXTRACTION_H_
#define PCL_FILTERS_EDGE_CONTOUR_EXTRACTION_H_

#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include "flann/flann.hpp"

namespace pcl
{
  /** \brief @ EdgeContourExtraction uses the Principal Component Analysis to extract object borders
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Di Gaetano, Mattia
    * \ingroup filters
    */
  template<typename PointT>
  class EdgeContourExtraction : public Filter<PointT>
  {
    using Filter<PointT>::input_;
    using Filter<PointT>::indices_;
    using Filter<PointT>::filter_name_;
    using Filter<PointT>::getClassName;

    using Filter<PointT>::removed_indices_;
    using Filter<PointT>::extract_removed_indices_;

   // typedef typename pcl::search::Search<PointT> KdTree;
   // typedef typename pcl::search::Search<PointT>::Ptr KdTreePtr;

	typedef typename pcl::KdTreeFLANN<PointT> KdTree;
	typedef typename pcl::KdTreeFLANN<PointT>::Ptr KdTreePtr;

    typedef typename Filter<PointT>::PointCloud PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    public:
      /** \brief Empty constructor. */
      EdgeContourExtraction (bool extract_removed_indices = false) :
        Filter<PointT>::Filter (extract_removed_indices), mean_k_ (10), eig_thresh_ (0.66), tree_ (), negative_ (false)
      {
        filter_name_ = "EdgeContourExtraction";
      }

      /** \brief Set the number of points (k) to use for PCA analysis.
        * \param nr_k the number of points to use for eigevalues estimation
        */
      inline void
      setEigMeanK (int nr_k)
      {
        mean_k_ = nr_k;
      }

      /** \brief Get the number of points to use for the PCA analysis. */
      inline int
      getEigMeanK ()
      {
        return (mean_k_);
      }

      /** \brief Set the principal eigenvalue threshold. All points lower than this value are removed.
        * \param eig_thresh the principal eigenvalue threshold (0 to 1)
        */
      inline void
      setPrincEigThresh (double eig_thresh)
      {
        eig_thresh_ = eig_thresh;
      }

      /** \brief Get the principal eigenvalue threshold as set by the user. */
      inline double
      getPrincEigThresh ()
      {
        return (eig_thresh_);
      }

      /** \brief Set whether the borders should be removed (true), or returned (false).
        * \param negative true if the borders should be removed, false otherwise
        */
      inline void
      setNegative (bool negative)
      {
        negative_ = negative;
      }

      /** \brief Get the value of the internal negative_ parameter. If
        * true, all points \e except the input indices will be returned.
        */
      inline bool
      getNegative ()
      {
        return (negative_);
      }

    protected:  
      /** \brief The number of points to use for mean distance estimation. */
      int mean_k_;

      /** \brief The principal eigenvalue threshold. */
      double eig_thresh_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief If false the borders will be returned (default: false). */
      bool negative_;

      /** \brief Apply the filter
        * \param output the resultant point cloud message
        */
      void
      applyFilter (PointCloud &output);
  };

  /** \brief @ EdgeContourExtraction uses the Principal Component Analysis to extract object borders
    * \note setFilterFieldName (), setFilterLimits (), and setFilterLimitNegative () are ignored.
    * \author Di Gaetano, Mattia
    * \ingroup filters
    */
  template<>
  class PCL_EXPORTS EdgeContourExtraction<sensor_msgs::PointCloud2> : public Filter<sensor_msgs::PointCloud2>
  {
    using Filter<sensor_msgs::PointCloud2>::filter_name_;
    using Filter<sensor_msgs::PointCloud2>::getClassName;

    using Filter<sensor_msgs::PointCloud2>::removed_indices_;
    using Filter<sensor_msgs::PointCloud2>::extract_removed_indices_;

    //typedef pcl::search::Search<pcl::PointXYZ> KdTree;
    //typedef pcl::search::Search<pcl::PointXYZ>::Ptr KdTreePtr;

	typedef pcl::KdTreeFLANN<pcl::PointXYZ> KdTree;
	typedef pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr KdTreePtr;

    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef PointCloud2::Ptr PointCloud2Ptr;
    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;

    public:
      /** \brief Empty constructor. */
      EdgeContourExtraction (bool extract_removed_indices = false) :
        Filter<sensor_msgs::PointCloud2>::Filter (extract_removed_indices), mean_k_ (10), eig_thresh_ (0.66), tree_ (), negative_ (false)
      {
        filter_name_ = "EdgeContourExtraction";
      }

      /** \brief Set the number of points (k) to use for PCA analysis.
      * \param nr_k the number of points to use for eigevalues estimation
      */
      inline void
      setEigMeanK (int nr_k)
      {
        mean_k_ = nr_k;
      }

      /** \brief Get the number of points to use for the PCA analysis. */
      inline int
      getEigMeanK ()
      {
        return (mean_k_);
      }

      /** \brief Set the principal eigenvalue threshold. All points lower than this value are removed.
        * \param eig_thresh the principal eigenvalue threshold (0 to 1)
        */
      inline void
      setPrincEigThresh (double std_mul)
      {
        eig_thresh_ = std_mul;
      }

      /** \brief Get the principal eigenvalue threshold as set by the user. */
      inline double
      getPrincEigThresh ()
      {
        return (eig_thresh_);
      }

      /** \brief Set whether the borders should be removed (true), or returned (false).
        * \param negative true if the borders should be removed, false otherwise
        */
      inline void
      setNegative (bool negative)
      {
        negative_ = negative;
      }

      /** \brief Get the value of the internal negative_ parameter. If
        * true, all points \e except the input indices will be returned.
        */
      inline bool
      getNegative ()
      {
        return (negative_);
      }

    protected:
      /** \brief The number of points to use for mean distance estimation. */
      int mean_k_;

      /** \brief The principal eigenvalue threshold. */
      double eig_thresh_;

      /** \brief A pointer to the spatial search object. */
      KdTreePtr tree_;

      /** \brief If false the borders will be returned (default: false). */
      bool negative_;

      /** \brief Apply the filter
        * \param output the resultant point cloud message
        */
      void
      applyFilter (PointCloud2 &output);
  };
}
#include "edge_contour_extraction.hpp"

#endif  //#ifndef PCL_FILTERS_EDGE_CONTOUR_EXTRACTION_H_
