#include "globaldef.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"
#include "octomap/Pointcloud.h"
#include "gicp/transform.h"
#include "ros/ros.h"
#include "timestamp.h"
#include "FileReader.h"
#include <cmath>

const double gl_gicp_epsilon = 1e-3;
const double gl_gicp_d_max_ = 5.0; // 10cm
const int gl_gicp_max_iterations = 50;
const int gl_gicp_min_point_cnt = 100;
const char* gl_pcd_file_dir = "/home/davidz/work/exprdata/pcds/conference_room";

const char* gl_bag_color_name = "/camera/rgb/image_color";
const char* gl_bag_depth_name = "/camera/depth/image";
const char* gl_bag_camera_info = "/camera/depth/camera_info";
const char* gl_bag_tf_msg = "/tf";

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

void fromColorPCL2OctoPC(color_point_cloud& pcl_pc, octomap::Pointcloud& oct_pc, octomap::point3d& ori_pose, vector<gl_color>& color)
{
    octomap::point3d oc_pt;
    for(int i=0;i<pcl_pc.points.size();i++)
    {
        color_point_type& pt = pcl_pc.points[i];
        if(!pcl_isfinite(pt.x) || \
                !pcl_isfinite(pt.y) || \
                !pcl_isfinite(pt.z))
            continue;
        oc_pt(0) = pt.x;
        oc_pt(1) = pt.y;
        oc_pt(2) = pt.z;
        oct_pc.push_back(oc_pt);
        color.push_back(gl_color(pt.r,pt.g,pt.b));
    }   
    Eigen::Vector4f& _pose = pcl_pc.sensor_origin_;
    ori_pose(0) = _pose[0];
    ori_pose(1) = _pose[1];
    ori_pose(2) = _pose[2];
}

void fromPCL2OctoPC(point_cloud& pcl_pc, octomap::Pointcloud& oct_pc, octomap::point3d& ori_pose)
{
    octomap::point3d oc_pt;
    for(int i=0;i<pcl_pc.points.size();i++)
    {
        point_type& pt = pcl_pc.points[i];
        if(!pcl_isfinite(pt.x) || \
                !pcl_isfinite(pt.y) || \
                !pcl_isfinite(pt.z))
            continue;
        oc_pt(0) = pt.x;
        oc_pt(1) = pt.y;
        oc_pt(2) = pt.z;
        oct_pc.push_back(oc_pt);
    }   
    Eigen::Vector4f& _pose = pcl_pc.sensor_origin_;
    ori_pose(0) = _pose[0];
    ori_pose(1) = _pose[1];
    ori_pose(2) = _pose[2];
}

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
    if(gicp_point_set->Size() > gl_gicp_min_point_cnt)
    {
        // build search structure for gicp:
        gicp_point_set->SetDebug(false); // true
        gicp_point_set->SetGICPEpsilon(gl_gicp_epsilon);
        gicp_point_set->BuildKDTree();
        gicp_point_set->ComputeMatrices();
        gicp_point_set->SetMaxIterationInner(8); // as in test_gicp->cpp
        gicp_point_set->SetMaxIteration(gl_gicp_max_iterations);
    }
    else
    {
        // ROS_WARN("GICP point set too small, this node will not be algined with GICP!");
        delete gicp_point_set;
        return NULL;
    }
    return gicp_point_set;
}

void fromRot2RPY(double& roll, double& pitch, double& yaw, Eigen::Matrix3f& m_ROT )
{
	// Pitch is in the range [-pi/2, pi/2 ], so this calculation is enough:
	pitch =  atan2( (double)(- m_ROT(2,0)), (double)hypot( m_ROT(0,0),m_ROT(1,0) ) ); //asin( - m_ROT(2,0) );

	// Roll:
	if ( (fabs(m_ROT(2,1))+fabs(m_ROT(2,2)))<10*std::numeric_limits<double>::epsilon() )
	{
		//Gimbal lock between yaw and roll. This one is arbitrarily forced to be zero.
		//Check http://reference.mrpt.org/svn/classmrpt_1_1poses_1_1_c_pose3_d.html. If cos(pitch)==0, the homogeneous matrix is:
		//When sin(pitch)==1:
		//  /0  cysr-sycr cycr+sysr x\   /0  sin(r-y) cos(r-y)  x\.
		//  |0  sysr+cycr sycr-cysr y| = |0  cos(r-y) -sin(r-y) y|
		//  |-1     0         0     z|   |-1    0         0     z|
		//  \0      0         0     1/   \0     0         0     1/
		//
		//And when sin(pitch)=-1:
		//  /0 -cysr-sycr -cycr+sysr x\   /0 -sin(r+y) -cos(r+y) x\.
		//  |0 -sysr+cycr -sycr-cysr y| = |0 cos(r+y)  -sin(r+y) y|
		//  |1      0          0     z|   |1    0          0     z|
		//  \0      0          0     1/   \0    0          0     1/
		//
		//Both cases are in a "gimbal lock" status. This happens because pitch is vertical.

		roll = 0.0;
		if (pitch>0) yaw=atan2((double)m_ROT(1,2),(double)m_ROT(0,2));
		else yaw=atan2((double)(-m_ROT(1,2)),(double)(-m_ROT(0,2)));
	}
	else
	{
		roll = atan2( (double)(m_ROT(2,1)), (double)m_ROT(2,2) );
		// Yaw:
		yaw = atan2( (double)(m_ROT(1,0)), (double)(m_ROT(0,0)) );
	}
}

void fromPose6d2Eigen(Eigen::Matrix4f& transformation, octomap::pose6d& pose)
{
    std::vector<double> rot(9);
    pose.rot().toRotMatrix(rot);
    for(int i=0;i<3;++i)
        for(int j=0;j<3;++j)
    {    
        transformation(i,j) = rot[i*3+j];
    }

    for(int i=0;i<3;i++)
        transformation(i,3) = pose.trans()(i);
    transformation(3,0) = transformation(3,1) = transformation(3,2) = 0;
    transformation(3,3) = 1;
}

void fromEigen2Pose6d(Eigen::Matrix4f& transformation, octomap::pose6d& pose)
{
    Eigen::Matrix3f m_ROT = transformation.block<3,3>(0, 0);
    Eigen::Vector3f translation = transformation.block<3,1>(0, 3); 
    // construct eular angle
    double roll,pitch,yaw;
    fromRot2RPY(roll,pitch,yaw,m_ROT);
    // construct translation x,y,z
    double x = translation(0); 
    double y = translation(1);
    double z = translation(2);
    pose = octomap::pose6d(x,y,z,roll,pitch,yaw);
}


void readPose(const char* fs, octomap::pose6d& pose)
{
    FILE* f = fopen(fs,"rb");
    if(f!=NULL)
    {
        double x,y,z,q0,q1,q2,q3;
        fscanf(f,"%lf %lf %lf %lf %lf %lf %lf",&x,&y,&z,&q0,&q1,&q2,&q3);
        pose = octomap::pose6d(octomath::Vector3(x,y,z), octomath::Quaternion(q0,q1,q2,q3));
        cout<<"read pose: "<<pose<<endl;
    }
    fclose(f);
}


