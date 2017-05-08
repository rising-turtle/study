#include "GicpWrapper.h"
#include "ros/ros.h"

CGicpWrapper::CGicpWrapper() 
   // m_bInited(false)
{}
CGicpWrapper::~CGicpWrapper(){}

void CGicpWrapper::Init()
{
    // Init GICP structure
    // m_bInited = true;
}

void CGicpWrapper::match_it(point_cloud& target, point_cloud& query, Eigen::Matrix4f& transform_, Eigen::Matrix4f& initTrans)
{
    dgc::gicp::GICPPointSet* gicp_target = fromPCL2GicpPC(target);
    if(gicp_target == NULL)
    {
        ROS_WARN("failed in CGicpWrapper::match_it!");
        return ;
    }
    dgc::gicp::GICPPointSet* gicp_query = fromPCL2GicpPC(query);
    if(gicp_query == NULL)
    {
        ROS_WARN("failed in CGicpWrapper::match_it!");
        delete gicp_target;       
        return ;
    }
    int iter = match_it(gicp_target, gicp_query, transform_, initTrans);
    if(iter < 0)
    {
        ROS_WARN("failed to match in CGicpWrapper::match_it!");
    }else
        ROS_INFO("succeed to match in CGip using iterations: %d", iter);
    delete gicp_target;
    delete gicp_query;
    return;
}

int CGicpWrapper::match_it(dgc::gicp::GICPPointSet* target, dgc::gicp::GICPPointSet* query, Eigen::Matrix4f& transform_, Eigen::Matrix4f& initTrans)
{
    dgc_transform_t initial;
    Eigen2GICP(initTrans,initial);

    dgc_transform_t final_trafo;
    dgc_transform_identity(final_trafo);
    int iterations;
    if(query->Size() > gl_gicp_min_point_cnt && 
            target->Size() > gl_gicp_min_point_cnt)
    {
        iterations = target->AlignScan(query, initial, final_trafo, gl_gicp_d_max_);
        GICP2Eigen(final_trafo,transform_);
    } else {
        ROS_WARN("GICP Point Sets not big enough. Skipping ICP");
        return -1;
    }
    return iterations;
}

void CGicpWrapper::Eigen2GICP(const Eigen::Matrix4f& m, dgc_transform_t g_m)
{
    for(int i=0;i<4; i++)
        for(int j=0;j<4; j++)
            g_m[i][j] = m(i,j);

}
void CGicpWrapper::GICP2Eigen(const dgc_transform_t g_m, Eigen::Matrix4f& m)
{
    for(int i=0;i<4; i++)
        for(int j=0;j<4; j++)
            m(i,j) = g_m[i][j];
}

void CGicpWrapper::gicpSetIdentity(dgc_transform_t m)
{
    for(int i=0;i<4; i++)
        for(int j=0;j<4; j++)
            if (i==j)
                m[i][j] = 1;
            else
                m[i][j] = 0;
}


