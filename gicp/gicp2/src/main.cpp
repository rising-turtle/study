#include "gicp.h"
#include "Eigen/Core"
#include "timestamp.h"
#include <iostream>
#include <fstream>
// #include <random>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <QtGui>
// #include <QApplication>
// #include <QObject>
// #include <octovis/ViewerGui.h>
// #include "RandomImpl.h"
#include <vector>
#include "PGicp.h"
#include "globaldef.h"

using namespace std;
void testGicp3();
void testGicp4();
extern int test_flann();

int main(int argc, char* argv[])
{
    // testGicp4();
    test_flann();
    cout<<"FINISHED!"<<endl;
    return 0;
}

void testGicp4()
{
    point_cloud_ptr targetPC(new point_cloud);
    pcl::io::loadPCDFile("person.pcd",*targetPC);
    point_cloud_ptr queryPC(new point_cloud(*targetPC));
    dgc::gicp::GICPPointSet* p0 = fromPCL2GicpPC(*targetPC);
    dgc::gicp::GICPPointSet* p1 = fromPCL2GicpPC(*queryPC);
    TestPGicp(p0, p1);
}

void testGicp3()
{
    point_cloud_ptr targetPC(new point_cloud);
    pcl::io::loadPCDFile("person.pcd",*targetPC);
    point_cloud_ptr queryPC(new point_cloud(*targetPC));

    dgc_transform_t t_base, t0, t1, t_est; 
    // set up the transformations
    dgc_transform_identity(t_base);
    dgc_transform_identity(t_est);
    dgc_transform_identity(t0);
    dgc_transform_identity(t1);
    dgc_transform_translate(t_base, 0.3, -0.1, 1);
    // dgc_transform_rotate_x(t_base, 0.2);
    // dgc_transform_rotate_y(t_base, -0.5);
    // dgc_transform_rotate_z(t_base, 0.3);
    
    dgc::gicp::GICPPointSet* p0 = fromPCL2GicpPC(*targetPC);
    dgc::gicp::GICPPointSet* p1 = fromPCL2GicpPC(*queryPC);
    
    cout<<"Start Counting..."<<endl;
    StartTiming();
    int iterations = p0->AlignScan(p1,t_base, t1, 5);
    double duration = StopTiming();
    cout<<"gicp cost: "<<duration<<" s"<<endl;
    // print the result
    cout << "Converged: " << endl;
    dgc_transform_print(t_base, "t_base");
    dgc_transform_print(t0, "t0");
    dgc_transform_print(t1, "t1");
    
    dgc_transform_left_multiply(t_est, t_base);
    dgc_transform_left_multiply(t_est, t1);
    dgc_transform_print(t_est, "final trans");
    cout << "Converged in " << iterations << " iterations." << endl;
    
}



/*

void normal_noise(octomap::pose6d& pose, double sigma_t, double sigma_r)
{
    static CRandomImpl rng;
    pose.x() += rng.normal(sigma_t);
    pose.y() += rng.normal(sigma_t);
    pose.z() += rng.normal(sigma_t);

    octomath::Vector3 euler = pose.rot().toEuler();
    euler(0) += rng.normal(sigma_r);
    euler(1) += rng.normal(sigma_r);
    euler(2) += rng.normal(sigma_r);

    pose.rot() = octomath::Quaternion(euler);
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

void calSqrErrorPose(octomap::pose6d& p1, octomap::pose6d& p2, double& t, double& r)
{
    octomap::pose6d trans = p1.inv()*p2;
    float x = trans.x();
    float y = trans.y();
    float z = trans.z();
    
    float roll = R2D(trans.roll());
    float pitch = R2D(trans.pitch());
    float yaw = R2D(trans.yaw());

    t = SQ(x) + SQ(y) + SQ(z);
    r = SQ(roll) + SQ(pitch) + SQ(yaw);
}
*/
