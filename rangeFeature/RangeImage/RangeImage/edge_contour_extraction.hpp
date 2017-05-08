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

#ifndef PCL_FILTERS_IMPL_EDGE_CONTOUR_EXTRACTION_H_
#define PCL_FILTERS_IMPL_EDGE_CONTOUR_EXTRACTION_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include "edge_contour_extraction.h"
#include <pcl/common/pca.h>
#include "preheader.h"
#include "MyPCA.h"

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT> void
pcl::EdgeContourExtraction<PointT>::applyFilter (PointCloud &output)
{

    if (input_->points.empty ())
    {
        output.width = output.height = 0;
        output.points.clear ();
        return;
    }

    // Initialize the spatial locator
    if (!tree_)
    {
       /* if (input_->isOrganized ())
            tree_.reset (new pcl::search::OrganizedNeighbor<PointT> ());
        else*/
       // tree_.reset (new pcl::search::KdTree<PointT> (false));
		tree_.reset(new pcl::KdTreeFLANN<PointT>);
    }

    // Send the input dataset to the spatial locator
    tree_->setInputCloud (input_);

    // Allocate enough space to hold the results
    std::vector<int> nn_indices (mean_k_);
    std::vector<float> nn_dists (mean_k_);

	pcl::MyPCA<PointT> pca;
    std::cout <<"n points "<< indices_->size() << std::endl;
    std::vector< float > eigval (indices_->size ());
    
    // Go over all the points and the principal eigenvalue
    for (size_t cp = 0; cp < indices_->size (); ++cp)
    {
      
        if (!pcl_isfinite (input_->points[(*indices_)[cp]].x) ||
                !pcl_isfinite (input_->points[(*indices_)[cp]].y) ||
                !pcl_isfinite (input_->points[(*indices_)[cp]].z))
        {
            eigval[cp] = 0;
            continue;
        }

        if (tree_->nearestKSearch ((*indices_)[cp], mean_k_, nn_indices, nn_dists) == 0)
        {
            eigval[cp] = 0;
            PCL_WARN ("[pcl::%s::applyFilter] Searching for the closest %d neighbors failed.\n", getClassName ().c_str (), mean_k_);
            continue;
        }

        // convert nn_indices to pcl::PointIndices for the pca module
        pcl::PointIndices::Ptr ptrInd (new pcl::PointIndices);
        ptrInd->header =input_->header;
        ptrInd->indices.insert(ptrInd->indices.begin(), nn_indices.begin(), nn_indices.end());

        // calculate the normalized principal eigenvalue
        if (nn_indices.size() > 2)
        {
            pca.setInputCloud(input_);
            pca.setIndices(ptrInd);
            eigval[cp] = (pca.getEigenValues()/(pca.getEigenValues()[0]+pca.getEigenValues()[1]+pca.getEigenValues()[2]) )[0];
        }
    }

    float eig_threshold;
    eig_threshold = eig_thresh_;
    
    output.points.resize (input_->points.size ());      // reserve enough space
    removed_indices_->resize (input_->points.size ());

    // Build a new cloud with edges and contours
    int nr_p = 0;
    int nr_removed_p = 0;

    for (int cp = 0; cp < static_cast<int> (indices_->size ()); ++cp)
    {
        if (negative_)
        {
            if (eigval[cp] > eig_threshold)
            {
                if (extract_removed_indices_)
                {
                    (*removed_indices_)[nr_removed_p] = cp;
                    nr_removed_p++;
                }
                continue;
            }
        }
        else
        {
            if (eigval[cp] <= eig_threshold)
            {    
                if (extract_removed_indices_)
                {
                    (*removed_indices_)[nr_removed_p] = cp;
                    nr_removed_p++;
                }
                continue;
            }
        }

        output.points[nr_p++] = input_->points[(*indices_)[cp]];
    }

    output.points.resize (nr_p);
    output.width  = nr_p;
    output.height = 1;
    output.is_dense = true; // nearestKSearch filters invalid points

    removed_indices_->resize (nr_removed_p);
	cout<<"negative_: "<<negative_<<endl;
	cout<<"extract_removed_indices_: "<<extract_removed_indices_<<endl;
	cout<<"ori point has: "<<input_->points.size()<<endl;
	cout<<"tar point has: "<<output.points.size()<<endl;
	cout<<"removied points: "<<nr_removed_p<<endl;
}

#define PCL_INSTANTIATE_EdgeContourExtraction(T) template class PCL_EXPORTS pcl::EdgeContourExtraction<T>;

#endif    // PCL_FILTERS_IMPL_EDGE_CONTOUR_EXTRACTION_H_

