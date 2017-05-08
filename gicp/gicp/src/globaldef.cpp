#include "globaldef.h"
#include <pcl/point_types.h>
#include <vector>
#include "timestamp.h"

int gl_max_iteration_inner_ = 5;
float epsilon_rot_ = 1e-2;
float epsilon_ = 1e-4;
int max_iteration_ = 10; // 20

dgc::gicp::GICPPointSet* fromPCL2GicpPC(point_cloud& pcl_pc)
{
    dgc::gicp::GICPPointSet* gicp_point_set = new dgc::gicp::GICPPointSet();

    dgc::gicp::GICPPoint g_p;
    g_p.range = -1;
    for(int k = 0; k < 3; k++) {
        for(int l = 0; l < 3; l++) {
            g_p.C[k][l] = (k == l)?1:0;
        }
    }

    std::vector<dgc::gicp::GICPPoint> non_NaN;
    non_NaN.reserve(pcl_pc.points.size());
    for (unsigned int i=0; i< pcl_pc.points.size(); i++ ){
        point_type&  p = pcl_pc.points.at(i);
        if (!isnan(p.z) && !isnan(p.x) && !isnan(p.y)) 
        { // add points to candidate pointset for icp
            g_p.x=p.x;
            g_p.y=p.y;
            g_p.z=p.z;
            non_NaN.push_back(g_p);
        }
    }
    for (int i=0; i<non_NaN.size(); i++ )
    {
        gicp_point_set->AppendPoint(non_NaN[i]);
    }
    if(gicp_point_set->Size() > 100)
    {
        // build search structure for gicp:
        gicp_point_set->SetDebug(false); // true
        gicp_point_set->SetGICPEpsilon(1e-3);
        gicp_point_set->BuildKDTree();
        gicp_point_set->ComputeMatrices();
        gicp_point_set->SetMaxIterationInner(gl_max_iteration_inner_); // as in test_gicp->cpp
        gicp_point_set->SetMaxIteration(max_iteration_);
    }
    else
    {
        delete gicp_point_set;
        return NULL;
    }
    return gicp_point_set;
}

static TTimeStamp gl_time_count_start;
static TTimeStamp gl_time_count_stop;

void StartTiming() // not thread-safe
{
    gl_time_count_start = getCurrentTime();
} 

double StopTiming()
{
    gl_time_count_stop = getCurrentTime();
    return timeDifference(gl_time_count_start, gl_time_count_stop);
}

void transformPoint(point_type& p_out, point_type& p_in, Eigen::Matrix4f& trans)
{
    Eigen::Matrix3f rot   = trans.block<3, 3> (0, 0);
    Eigen::Vector3f trans_ = trans.block<3, 1> (0, 3);
    p_out.getVector3fMap() = rot * p_in.getVector3fMap() + trans_;
}

void fromdgcTrans2Eigen(dgc_transform_t t_in, Eigen::Matrix4f& t_out)
{
    for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
    t_out(i,j) = t_in[i][j];
}

void fromEigen2dgcTrans(Eigen::Matrix4f& t_in, dgc_transform_t t_out)
{
    for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
    t_out[i][j] = t_in(i,j);
}

