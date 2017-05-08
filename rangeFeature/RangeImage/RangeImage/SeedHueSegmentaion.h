/*
00002  * Software License Agreement (BSD License)
00003  *
00004  *  Point Cloud Library (PCL) - www.pointclouds.org
00005  *  Copyright (c) 2010-2011, Willow Garage, Inc.
00006  *
00007  *  All rights reserved.
00008  *
00009  *  Redistribution and use in source and binary forms, with or without
00010  *  modification, are permitted provided that the following conditions
00011  *  are met:
00012  *
00013  *   * Redistributions of source code must retain the above copyright
00014  *     notice, this list of conditions and the following disclaimer.
00015  *   * Redistributions in binary form must reproduce the above
00016  *     copyright notice, this list of conditions and the following
00017  *     disclaimer in the documentation and/or other materials provided
00018  *     with the distribution.
00019  *   * Neither the name of Willow Garage, Inc. nor the names of its
00020  *     contributors may be used to endorse or promote products derived
00021  *     from this software without specific prior written permission.
00022  *
00023  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
00024  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
00025  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
00026  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
00027  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
00028  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
00029  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
00030  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
00031  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
00032  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
00033  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
00034  *  POSSIBILITY OF SUCH DAMAGE.
00035  *
00036  * $id: $
00037  */

 #ifndef PCL_SEEDED_HUE_SEGMENTATION_H_
 #define PCL_SEEDED_HUE_SEGMENTATION_H_
 
 #include <pcl/pcl_base.h>
 #include <pcl/point_types_conversion.h>
 //#include <pcl/search/pcl_search.h>
 #include  "pcl/kdtree/kdtree_flann.h"
 
 namespace pcl
 {
	 
		   void 
		   seededHueSegmentation (const PointCloud<PointXYZRGB>                           &cloud, 
		                          const boost::shared_ptr<pcl::KdTreeFLANN<PointXYZRGB> >   &tree, 
		                          float                                                   tolerance, 
		                          PointIndices                                            &indices_in, 
		                          PointIndices                                            &indices_out, 
		                          float                                                   delta_hue = 0.0);
	 
		   void 
		   seededHueSegmentation (const PointCloud<PointXYZRGB>                           &cloud, 
		                          const boost::shared_ptr<pcl::KdTreeFLANN<PointXYZRGBL> >  &tree, 
		                          float                                                   tolerance, 
		                          PointIndices                                            &indices_in, 
		                          PointIndices                                            &indices_out, 
		                          float                                                   delta_hue = 0.0);
	 
		 
		   class SeededHueSegmentation: public PCLBase<PointXYZRGB>
		   {
			     typedef PCLBase<PointXYZRGB> BasePCLBase;
			 
				     public:
			       typedef pcl::PointCloud<PointXYZRGB> PointCloud;
			       typedef PointCloud::Ptr PointCloudPtr;
			       typedef PointCloud::ConstPtr PointCloudConstPtr;
			 
				   //typedef pcl::search::Search<PointXYZRGB> KdTree;
			       //typedef pcl::search::Search<PointXYZRGB>::Ptr KdTreePtr;
					
				   typedef pcl::KdTreeFLANN<PointXYZRGB> KdTree;
				   typedef pcl::KdTreeFLANN<PointXYZRGB>::Ptr KdTreePtr;

				       typedef PointIndices::Ptr PointIndicesPtr;
			       typedef PointIndices::ConstPtr PointIndicesConstPtr;
			 
				 
				       SeededHueSegmentation () : tree_ (), cluster_tolerance_ (0), delta_hue_ (0.0)
				       {};
			 
				       inline void 
				       setSearchMethod (const KdTreePtr &tree) { tree_ = tree; }
			 
				       inline KdTreePtr 
				       getSearchMethod () const { return (tree_); }
			 
				       inline void 
				       setClusterTolerance (double tolerance) { cluster_tolerance_ = tolerance; }
			 
				       inline double 
				       getClusterTolerance () const { return (cluster_tolerance_); }
			 
				       inline void 
				       setDeltaHue (float delta_hue) { delta_hue_ = delta_hue; }
			 
				       inline float 
				       getDeltaHue () const { return (delta_hue_); }
			 
				       void 
				       segment (PointIndices &indices_in, PointIndices &indices_out);
			 
				     protected:
			       // Members derived from the base class
				       using BasePCLBase::input_;
			       using BasePCLBase::indices_;
			       using BasePCLBase::initCompute;
			       using BasePCLBase::deinitCompute;
			 
				       KdTreePtr tree_;
			 
				       double cluster_tolerance_;
			 
				       float delta_hue_;
			 
				       virtual std::string getClassName () const { return ("seededHueSegmentation"); }
			   };
			 }
 
 #endif  //#ifndef PCL_SEEDED_HUE_SEGMENTATION_H_