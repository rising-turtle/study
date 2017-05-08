#include "Submap.h"

CSubmap::CSubmap():
    m_bHasReduced(false),
    // pc_col(new color_point_cloud),
    pc_col(new point_cloud),
    m_pOctoTree(new ColorOctreeImpl)
{}

CSubmap::~CSubmap()
{
    if(m_pOctoTree) delete m_pOctoTree;
}

template<typename NODE>
void CSubmap<NODE>::addNode(vector<NODE*>& nodes)
{
    for(int i=0;i<nodes.size();i++)
        addNode(nodes[i]);
}

template<typename NODE>
void CSubmap<NODE>::addNode(NODE* node)
{
    // 1 trajectory
    // 2 features
    // 3 point cloud
    m_pOctoTree->insertPointCloud(*(node->pc_col));
}

template<typename NODE>
void CSubmap<NODE>::reduction()
{
    if(m_bHasReduced)
    {
        cout<<"This submap has been Reduced!"<<endl;
        return ;
    }
    m_bHasReduced = true;
    // 1 trajectory
    // 2 features
    // 3 point cloud
    int maxDepth = 16;
    for(ColorOctreeImpl::leaf_iterator it = m_pOctoTree->begin(maxDepth), end=m_pOctoTree->end(); it!= end; ++it) 
    {
        if(!m_pOctoTree->isNodeOccupied(*it))
        {
            octomap::point3d pt = it.getCoordinate();
            pc_col->points.push_back(point_type(pt(0),pt(1),pt(2)));
            //     list_iterator.push_back(OcTreeVolume(it.getCoordinate(), it.getSize()));
        }
    }
    pc_col->width = pc_col->points.size();
    pc_col->height = 1;
}
