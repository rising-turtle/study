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
 * $Id: octree_search.h 5596 2012-04-17 15:09:31Z jkammerl $
 */

#ifndef PCL_OCTREE_SEARCH_H_
#define PCL_OCTREE_SEARCH_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "octree_pointcloud.h"

#include "octree_base.h"
#include "octree2buf_base.h"
#include "octree_nodes.h"

namespace pcl
{
  namespace octree
  {
    template<typename PointT, typename LeafT = OctreeLeafDataTVector<int> , typename OctreeT = OctreeBase<int, LeafT> >
    class OctreePointCloudSearch : public OctreePointCloud<PointT, LeafT, OctreeT>
    {
      public:
        // public typedefs
        typedef boost::shared_ptr<std::vector<int> > IndicesPtr;
        typedef boost::shared_ptr<const std::vector<int> > IndicesConstPtr;

        typedef pcl::PointCloud<PointT> PointCloud;
        typedef boost::shared_ptr<PointCloud> PointCloudPtr;
        typedef boost::shared_ptr<const PointCloud> PointCloudConstPtr;

        // public typedefs for single/double buffering
        typedef OctreePointCloudSearch<PointT, LeafT, OctreeBase<int, LeafT> > SingleBuffer;
        typedef OctreePointCloudSearch<PointT, LeafT, Octree2BufBase<int, LeafT> > DoubleBuffer;
        typedef OctreePointCloudSearch<PointT, LeafT, OctreeLowMemBase<int, LeafT> > LowMem;

        // Boost shared pointers
        typedef boost::shared_ptr<OctreePointCloudSearch<PointT, LeafT, OctreeT> > Ptr;
        typedef boost::shared_ptr<const OctreePointCloudSearch<PointT, LeafT, OctreeT> > ConstPtr;

        // Eigen aligned allocator
        typedef std::vector<PointT, Eigen::aligned_allocator<PointT> > AlignedPointTVector;

        typedef typename OctreeT::OctreeBranch OctreeBranch;
        typedef typename OctreeT::OctreeLeaf OctreeLeaf;

        OctreePointCloudSearch (const double resolution) :
          OctreePointCloud<PointT, LeafT, OctreeT> (resolution)
        {
        }

        virtual
        ~OctreePointCloudSearch ()
        {
        }

        bool
        voxelSearch (const PointT& point, std::vector<int>& pointIdx_data);

        bool
        voxelSearch (const int index, std::vector<int>& pointIdx_data);

        inline int
        nearestKSearch (const PointCloud &cloud, int index, int k, std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances)
        {
          return (nearestKSearch (cloud[index], k, k_indices, k_sqr_distances));
        }

        int
        nearestKSearch (const PointT &p_q, int k, std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances);

        int
        nearestKSearch (int index, int k, std::vector<int> &k_indices,
                        std::vector<float> &k_sqr_distances);

        inline void
        approxNearestSearch (const PointCloud &cloud, int query_index, int &result_index,
                             float &sqr_distance)
        {
          return (approxNearestSearch (cloud.points[query_index], result_index, sqr_distance));
        }

        void
        approxNearestSearch (const PointT &p_q, int &result_index, float &sqr_distance);

        void
        approxNearestSearch (int query_index, int &result_index, float &sqr_distance);

        int
        radiusSearch (const PointCloud &cloud, int index, double radius,
                      std::vector<int> &k_indices, std::vector<float> &k_sqr_distances,
                      unsigned int max_nn = 0)
        {
          return (radiusSearch (cloud.points[index], radius, k_indices, k_sqr_distances, max_nn));
        }

        int
        radiusSearch (const PointT &p_q, const double radius, std::vector<int> &k_indices,
                      std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const;

        int
        radiusSearch (int index, const double radius, std::vector<int> &k_indices,
                      std::vector<float> &k_sqr_distances, unsigned int max_nn = 0) const;

        int
        getIntersectedVoxelCenters (Eigen::Vector3f origin, Eigen::Vector3f direction,
                                    AlignedPointTVector &voxelCenterList) const;

        int
        getIntersectedVoxelIndices (Eigen::Vector3f origin, Eigen::Vector3f direction,
                                    std::vector<int> &k_indices) const;


        int
        boxSearch (const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt, std::vector<int> &k_indices) const;

      protected:
        // Octree-based search routines & helpers

        class prioBranchQueueEntry
        {
          public:
            prioBranchQueueEntry () : node (), pointDistance (0), key ()
            {
            }

            prioBranchQueueEntry (OctreeNode* _node, OctreeKey& _key, float _point_distance) :
              node (_node), pointDistance (_point_distance), key (_key)
            {
            }

            bool
            operator < (const prioBranchQueueEntry rhs) const
            {
              return (this->pointDistance > rhs.pointDistance);
            }

            const OctreeNode* node;

            float pointDistance;

            OctreeKey key;
        };


        class prioPointQueueEntry
        {
          public:

            prioPointQueueEntry () :
              pointIdx_ (0), pointDistance_ (0)
            {
            }

            prioPointQueueEntry (unsigned int& pointIdx, float pointDistance) :
              pointIdx_ (pointIdx), pointDistance_ (pointDistance)
            {
            }

            bool
            operator< (const prioPointQueueEntry& rhs) const
            {
              return (this->pointDistance_ < rhs.pointDistance_);
            }

            int pointIdx_;

            float pointDistance_;
        };

        float
        pointSquaredDist (const PointT& pointA, const PointT& pointB) const;

        // Recursive search routine methods

        void
        getNeighborsWithinRadiusRecursive (const PointT& point, const double radiusSquared,
                                           const OctreeBranch* node, const OctreeKey& key,
                                           unsigned int treeDepth, std::vector<int>& k_indices,
                                           std::vector<float>& k_sqr_distances, unsigned int max_nn) const;

        double
        getKNearestNeighborRecursive (const PointT& point, unsigned int K, const OctreeBranch* node,
                                      const OctreeKey& key, unsigned int treeDepth,
                                      const double squaredSearchRadius,
                                      std::vector<prioPointQueueEntry>& pointCandidates) const;

        void
        approxNearestSearchRecursive (const PointT& point, const OctreeBranch* node, const OctreeKey& key,
                                      unsigned int treeDepth, int& result_index, float& sqr_distance);

        int
        getIntersectedVoxelCentersRecursive (double minX, double minY, double minZ, double maxX, double maxY,
                                             double maxZ, unsigned char a, const OctreeNode* node,
                                             const OctreeKey& key, AlignedPointTVector &voxelCenterList) const;


        void
        boxSearchRecursive (const Eigen::Vector3f &min_pt, const Eigen::Vector3f &max_pt, const OctreeBranch* node,
                            const OctreeKey& key, unsigned int treeDepth, std::vector<int>& k_indices) const;

        int
        getIntersectedVoxelIndicesRecursive (double minX, double minY, double minZ,
                                             double maxX, double maxY, double maxZ,
                                             unsigned char a, const OctreeNode* node, const OctreeKey& key,
                                             std::vector<int> &k_indices) const;

        inline void
        initIntersectedVoxel (Eigen::Vector3f &origin, Eigen::Vector3f &direction,
                              double &minX, double &minY, double &minZ,
                              double &maxX, double &maxY, double &maxZ,
                              unsigned char &a) const
        {
          // Account for division by zero when direction vector is 0.0
          const float epsilon = 1e-10f;
          if (direction.x () == 0.0)
            direction.x () = epsilon;
          if (direction.y () == 0.0)
            direction.y () = epsilon;
          if (direction.z () == 0.0)
            direction.z () = epsilon;

          // Voxel childIdx remapping
          a = 0;

          // Handle negative axis direction vector
          if (direction.x () < 0.0)
          {
            origin.x () = static_cast<float> (this->minX_) + static_cast<float> (this->maxX_) - origin.x ();
            direction.x () = -direction.x ();
            a |= 4;
          }
          if (direction.y () < 0.0)
          {
            origin.y () = static_cast<float> (this->minY_) + static_cast<float> (this->maxY_) - origin.y ();
            direction.y () = -direction.y ();
            a |= 2;
          }
          if (direction.z () < 0.0)
          {
            origin.z () = static_cast<float> (this->minZ_) + static_cast<float> (this->maxZ_) - origin.z ();
            direction.z () = -direction.z ();
            a |= 1;
          }
          minX = (this->minX_ - origin.x ()) / direction.x ();
          maxX = (this->maxX_ - origin.x ()) / direction.x ();
          minY = (this->minY_ - origin.y ()) / direction.y ();
          maxY = (this->maxY_ - origin.y ()) / direction.y ();
          minZ = (this->minZ_ - origin.z ()) / direction.z ();
          maxZ = (this->maxZ_ - origin.z ()) / direction.z ();
        }

        inline int
        getFirstIntersectedNode (double minX, double minY, double minZ, double midX, double midY, double midZ) const
        {
          int currNode = 0;

          if (minX > minY)
          {
            if (minX > minZ)
            {
              // max(minX, minY, minZ) is minX. Entry plane is YZ.
              if (midY < minX)
                currNode |= 2;
              if (midZ < minX)
                currNode |= 1;
            }
            else
            {
              // max(minX, minY, minZ) is minZ. Entry plane is XY.
              if (midX < minZ)
                currNode |= 4;
              if (midY < minZ)
                currNode |= 2;
            }
          }
          else
          {
            if (minY > minZ)
            {
              // max(minX, minY, minZ) is minY. Entry plane is XZ.
              if (midX < minY)
                currNode |= 4;
              if (midZ < minY)
                currNode |= 1;
            }
            else
            {
              // max(minX, minY, minZ) is minZ. Entry plane is XY.
              if (midX < minZ)
                currNode |= 4;
              if (midY < minZ)
                currNode |= 2;
            }
          }

          return currNode;
        }

        inline int
        getNextIntersectedNode (double x, double y, double z, int a, int b, int c) const
        {
          if (x < y)
          {
            if (x < z)
              return a;
            else
              return c;
          }
          else
          {
            if (y < z)
              return b;
            else
              return c;
          }

          return 0;
        }

      };
  }
}

#define PCL_INSTANTIATE_OctreePointCloudSearch(T) template class PCL_EXPORTS pcl::octree::OctreePointCloudSearch<T>;

#endif    // PCL_OCTREE_SEARCH_H_