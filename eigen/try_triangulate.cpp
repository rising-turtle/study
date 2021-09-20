/*
test the triangulate function 

*/


#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <string>
#include <cmath>

using namespace std; 
using namespace Eigen;

void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &point_3d)
{
    Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
    design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
    design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
    design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
    design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
    Eigen::Vector4d triangulated_point;
    triangulated_point =
              design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
    point_3d(0) = triangulated_point(0) / triangulated_point(3);
    point_3d(1) = triangulated_point(1) / triangulated_point(3);
    point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

int main()
{
    Vector2d point0(-0.0013, -0.222498); 
    Vector2d point1(-0.0417136, -0.224929); 
    Eigen::Matrix<double, 3, 4> leftPose = Eigen::Matrix<double, 3, 4>::Zero();
    leftPose.leftCols<3>() = Eigen::Matrix3d::Identity(); 

    Eigen::Matrix<double, 3, 4> rightPose = Eigen::Matrix<double, 3, 4>::Zero(); 
    rightPose.leftCols<3>() = Eigen::Matrix3d::Identity(); 

    Eigen::Matrix3d R; 
    R<< 0.9998053, 0.01119774, 0.01624713,
          -0.01114776, 0.99993286, -0.00316357, 
          -0.01628147, 0.00298183, 0.999863; 
    // rightPose.leftCols<3>() = R; 

    rightPose.rightCols<1>() = Eigen::Vector3d(-0.08, 0, 0); // 0.00074435, 0.00044255);

    Vector3d pt3d; 
    triangulatePoint(leftPose, rightPose, point0, point1, pt3d); 

    cout <<"pt3d: "<<pt3d.transpose()<<endl; 
    
    return 0 ; 

}







