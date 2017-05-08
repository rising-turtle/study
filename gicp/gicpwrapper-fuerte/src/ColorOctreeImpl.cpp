#include "ColorOctreeImpl.h"
#include "ros/ros.h"

using namespace octomap;

ColorOctreeImpl::ColorOctreeImpl(float res):ColorOcTree(res){}
ColorOctreeImpl::~ColorOctreeImpl(){}
gl_color ColorOctreeImpl::s_octree_color(getRed());
void ColorOctreeImpl::setOctreeColor(gl_color& new_color){ColorOctreeImpl::s_octree_color = new_color;}

void ColorOctreeImpl::insertPointCloud(const Pointcloud& oct_pc, const octomap::point3d& sensor_origin,
        double maxrange, bool lazy_eval)
{
    ROS_WARN("in the devired class function!");
    KeySet free_cells, occupied_cells;
    computeUpdate(oct_pc, sensor_origin, free_cells, occupied_cells, maxrange);    
    for (KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
        updateNode(*it, false, lazy_eval);
    }
    for (KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
        updateNode(*it, true, lazy_eval);
        setNodeColor(*it, s_octree_color.r_, s_octree_color.g_, s_octree_color.b_ );
        // ColorOcTreeNode* pNode = search(*it);
        // printf("input color is %d, %d, %d\n", pNode->getColor().r, pNode->getColor().g, pNode->getColor().b);
    }
}

void ColorOctreeImpl::insertPointCloud(const octomap::Pointcloud& oct_pc, const octomap::point3d& ori_pose, const octomap::pose6d& frame_origin, double maxrange, bool lazy_eval)
{
        Pointcloud transformed_scan (oct_pc);
        transformed_scan.transform(frame_origin);
        point3d transformed_sensor_origin = frame_origin.transform(ori_pose);
        insertPointCloud(transformed_scan, transformed_sensor_origin, maxrange, lazy_eval);
}
