/*
    Beacause coordinate system is defined differently, so the [roll, pitch, yaw]->R, and pi= Rij*pj may not be right if roll is not rotate around x axis, pitch around y axis, and yaw around z axis. 
    So, if use euler angle to represents rotation, needs to separately defines R * p = Rroll * Rpitch * Ryaw * p 
    draw crazy
*/

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <iostream>
#include <cmath>
#include "utility.h"

#define D2R(d) (((d)*M_PI)/180.)

using namespace std;

int main()
{
    double dr = 10; 
    double dp = 20; 
    double dy = 30;
    double roll = D2R(dr); 
    double pitch = D2R(dp); 
    double yaw = D2R(dy); 
    
    // Eigen::Vector3d ypr(dy, dp, dr); 
    // Eigen::Vector3d ypr(dp, dy, dr); 
    Eigen::Vector3d yrp(-dr, dp, dy); 
    Eigen::Matrix<double, 3, 3> R = Utility::yrp2R(yrp); 
    
    cout <<"R: "<<endl<<R<<endl; 
    // Eigen::Vector3d ypr2 = Utility::R2ypr(R); 
    // cout<<"ypr2: "<<endl<<ypr2<<endl; 

    Eigen::Vector3d p(0.1, 0.2, 0.3); 
    Eigen::Vector3d tp = R * p; 
    cout <<"R*p = "<<endl<<tp<<endl; 
 /*   
    Eigen::Quaterniond q1(R); 
    Eigen::Quaterniond q2(q1.w(), -q1.x(), -q1.y(), q1.z()); 
    Eigen::Matrix<double, 3, 3> R2 = q2.toRotationMatrix(); 
    Eigen::Vector3d tp2 = R2 * p; 
    cout <<"R2 *p = "<<endl<<tp2<<endl; 

    Eigen::Vector3d rpy(dr, dp, dy); 
    Eigen::Matrix<double, 3, 3> R3 = Utility::rpy2R(rpy); 
    Eigen::Vector3d tp3 = R3*p; 
    cout <<"R3 * p = "<<endl<<tp3<<endl;
*/
    // yaw corresponding to y axis 
    double tx = p(0); double ty = p(1); double tz = p(2); 
    double x1 = cos(yaw) * tx + sin(yaw) * tz;
    double y1 = ty;
    double z1 = -sin(yaw) * tx + cos(yaw) * tz;

    // pitch corresponding to x axis 
    double x2 = x1;
    double y2 = cos(pitch) * y1 - sin(pitch) * z1;
    double z2 = sin(pitch) * y1 + cos(pitch) * z1;

    // roll corresponding to z axis 
    tx = cos(roll) * x2 + sin(roll) * y2;
    ty = -sin(roll) * x2 + cos(roll) * y2;
    tz = z2;
    cout <<"multiply: "<<tx<<" "<<ty<<" "<<tz<<endl;
}


