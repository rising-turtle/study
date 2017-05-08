#ifndef COLOR_OCTREE_IMPL_H
#define COLOR_OCTREE_IMPL_H

#include "globaldef.h"
#include "octomap/ColorOcTree.h"
#include "octomap/Pointcloud.h"

class ColorOctreeImpl : public octomap::ColorOcTree
{
public:
    ColorOctreeImpl(float res=0.1);
    virtual ~ColorOctreeImpl();
    virtual void insertPointCloud(const octomap::Pointcloud& , const octomap::point3d& , double maxrange=-1., bool lazy_eval = false);
    virtual void insertPointCloud(const octomap::Pointcloud& , const octomap::point3d& , const octomap::pose6d&, double maxrange = -1., bool lazy_eval = false);
    // virtual void insertPCLPointcloud(pcl::PointCloud<pcl::PointXYZRGB>& , double maxrange=-1., bool lazy_eval = false);
    static void setOctreeColor(gl_color&);
    static gl_color s_octree_color;

};


#endif
