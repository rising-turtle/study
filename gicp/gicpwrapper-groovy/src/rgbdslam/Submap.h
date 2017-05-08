#ifndef SUBMAP_H
#define SUBMAP_H

#include <iostream>
#include <vector>
#include "node.h"

using namespace std;

template<typename NODE>
class CSubmap 
{
public:
    CSubmap();
    ~CSubmap();
    void addNode(vector<NODE*>& );
    void addNode(NODE*);
    
    // 1 trajectory
    // 2 overlapped features
    
    // How to update feature? according to sigma of the feature points? 
    vector<NODE*> m_nodes;
    // 3 point cloud
    ColorOctreeImpl* m_pOctoTree;   

    // reduction
    bool m_bHasReduced;
    // color_pc_ptr pc_col;
    point_cloud_ptr pc_col;
private:
    CSubmap(const CSubmap&);
    CSubmap& operator=(const CSubmap&);
};

#endif
