/*
    test the distance between a point to plane 

*/

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
using namespace std; 


double determinant(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3);

int main()
{
     Eigen::Vector3d p1(1, 2, 3); 
    Eigen::Vector3d p2(0.5, 1.0, 2.0); 
    Eigen::Vector3d p3(3, 0.2, -1); 

//    Eigen::Vector3d p1(1, 2, 10); 
//    Eigen::Vector3d p2(0.5, 1.0, 10); 
//    Eigen::Vector3d p3(3, 0.2, 10); 
// 

    double x1 = p1(0); double y1 = p1(1); double z1 = p1(2); 
    double x2 = p2(0); double y2 = p2(1); double z2 = p2(2); 
    double x3 = p3(0); double y3 = p3(1); double z3 = p3(2); 

    Eigen::Vector3d p0(0.3, -0.7, 1.); 
    double u = 0.3; double v = -0.7; 

    double depth = (x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1)/ (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2 + u*y1*z2 - u*y2*z1
- v*x1*z2 + v*x2*z1 - u*y1*z3 + u*y3*z1 + v*x1*z3 - v*x3*z1 + u*y2*z3 - u*y3*z2 - v*x2*z3 + v*x3*z2);
    cout <<"depth: "<<depth<<endl; 
    
    Eigen::Vector3d p21 = p2 - p1; 
    Eigen::Vector3d p31 = p3 - p1; 
    Eigen::Vector3d n = p21.cross(p31); 
    Eigen::Vector3d n_norm = n/n.norm(); 
    cout <<"n_norm: "<<endl<<n_norm<<endl;
    // Eigen::Vector3d p01 = p0 - p1; 
    depth = n_norm.dot(p1)/n_norm.dot(p0); 
    cout <<"depth: "<<depth<<endl; 

    double A0 = determinant(p1, p2, p3); 
    double A1 = determinant(p1, p2, p0); 
    double A2 = determinant(p2, p3, p0); 
    double A3 = determinant(p3, p1, p0); 

    double s = A0/(A1+A2+A3);
    cout<<"A0 "<<A0<<" A1 "<<A1<<" A2 "<<A2<<" A3 "<<A3<<endl;
    cout <<"s = "<<s<<endl;


    return 1;
}



double determinant(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3)
{
    Eigen::Matrix<double, 3, 3> A; 
    A.col(0) = p1;
    A.col(1) = p2; 
    A.col(2) = p3; 
    // Eigen::Matrix<double, 3, 3> At = A.transpose(); 
    return A.determinant(); 
}

