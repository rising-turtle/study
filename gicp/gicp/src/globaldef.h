#ifndef GLOBALDEF_H
#define GLOBALDEF_H
#include <pcl/point_types.h>
#include <gicp.h>
#include "Eigen/Core"

//typedef pcl::PointXYZRGB point_type;
typedef pcl::PointXYZ point_type;
typedef pcl::PointCloud<point_type> point_cloud;
typedef pcl::PointCloud<point_type>::Ptr point_cloud_ptr;
typedef pcl::PointCloud<point_type>::ConstPtr point_cloud_cptr;

void StartTiming();
double StopTiming();
dgc::gicp::GICPPointSet* fromPCL2GicpPC(point_cloud& pcl_pc);

void transformPoint(point_type& p_out, point_type& p_in, Eigen::Matrix4f& trans);
void fromdgcTrans2Eigen(dgc_transform_t t_in, Eigen::Matrix4f& t_out);
void fromEigen2dgcTrans(Eigen::Matrix4f& t_in, dgc_transform_t t_out);

extern int gl_max_iteration_inner_;
extern float epsilon_rot_;
extern float epsilon_;
extern int max_iteration_;

#endif
