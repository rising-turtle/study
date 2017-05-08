#ifndef GICP_WRAPPER_H
#define GICP_WRAPPER_H

#include "globaldef.h"
#include "gicp/gicp.h"
#include "gicp/transform.h"
#include <Eigen/Core>

class CGicpWrapper
{
public:
    CGicpWrapper();
    virtual ~CGicpWrapper();
    void Init();
    virtual void match_it(point_cloud& , point_cloud&, Eigen::Matrix4f& , Eigen::Matrix4f&);
    virtual int match_it(dgc::gicp::GICPPointSet*, dgc::gicp::GICPPointSet*, Eigen::Matrix4f&, Eigen::Matrix4f&);

    void Eigen2GICP(const Eigen::Matrix4f&, dgc_transform_t);
    void GICP2Eigen(const dgc_transform_t, Eigen::Matrix4f& );
    void gicpSetIdentity(dgc_transform_t);
};

#endif 
