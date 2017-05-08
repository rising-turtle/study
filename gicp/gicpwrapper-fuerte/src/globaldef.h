#ifndef GLOBAL_DEF_H
#define GLOBAL_DEF_H
#include "signalslib.hpp"
#include <octomap/octomap_types.h>
#include <pcl/point_types.h>
#include "gicp/gicp.h"
#include "colortable.h"
#include "Eigen/Core"
#include <vector>
namespace octomap
{
    class Pointcloud;
}

// RAD2DEG
#define R2D(x) ((180.0*(x))/M_PI)
#define D2R(x) ((M_PI*(x))/180.0)

// SQUARE
#define SQ(x) ((x)*(x))

//typedef pcl::PointXYZRGB point_type;
//typedef pcl::PointXYZ point_type;
typedef pcl::PointXYZRGBA point_type;
typedef pcl::PointCloud<point_type> point_cloud;
typedef pcl::PointCloud<point_type>::Ptr point_cloud_ptr;
typedef pcl::PointCloud<point_type>::ConstPtr point_cloud_cptr;

// gicp parameters
extern const double gl_gicp_epsilon;
extern const double gl_gicp_d_max_; // 10cm
extern const int gl_gicp_max_iterations;
extern const int gl_gicp_min_point_cnt;

// pcd file dir
extern const char* gl_pcd_file_dir;

// bag info 
extern const char* gl_bag_color_name;
extern const char* gl_bag_depth_name;
extern const char* gl_bag_camera_info;
extern const char* gl_bag_tf_msg;

extern void fromPCL2OctoPC(point_cloud& , octomap::Pointcloud&, octomap::point3d& );
extern dgc::gicp::GICPPointSet* fromPCL2GicpPC(point_cloud&);
extern void fromEigen2Pose6d(Eigen::Matrix4f& , octomap::pose6d&);
extern void fromPose6d2Eigen(Eigen::Matrix4f& , octomap::pose6d&);
extern void fromRot2RPY(double&, double&, double&, Eigen::Matrix3f&);

extern void StartTiming();
extern double StopTiming();

extern void readPcdAndPose(const char*, std::vector<point_cloud_ptr>& , std::vector<octomap::pose6d>&, int num = -1, int step = 5);

#endif
