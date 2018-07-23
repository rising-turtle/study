#include "Eigen/Core"
#include "Eigen/Geometry"


#include <vector>
#include <iostream>
#include <cmath>

using namespace std; 

#define D2R(d) (((d)*M_PI)/180.)
#define R2D(r) (((r)*180.)/M_PI)

double sinc(double x)
{
    return sin(x)/(x);
}

int main(int argc, char* argv[])
{
    double psi, phi, theta; 
    psi = D2R(28.7); 
    phi = D2R(10.3); 
    theta = D2R(41.2); 
    Eigen::Matrix<double, 3,3> Rw2u; 
    double cpsi = cos(psi);  double spsi = sin(psi); 
    double cphi = cos(phi);  double sphi = sin(phi); 
    double ctheta = cos(theta);  double stheta = sin(theta); 
    Rw2u << cpsi*cphi - spsi*stheta*sphi, -spsi*ctheta, cpsi*sphi + spsi*stheta*cphi, 
	    spsi*cphi + cpsi*stheta*sphi,  cpsi*ctheta, spsi*sphi - cpsi*stheta*cphi,
	    -ctheta*sphi,		   stheta,	ctheta*cphi; 
    Eigen::Matrix<double, 3,3> Ru2w = Rw2u.inverse(); 
    Eigen::Vector3d g(0, 0, -9.81);
    Eigen::Vector3d a = Ru2w * g;
    cout <<"Rw2u: "<<endl<<Rw2u<<endl; 
    cout <<"Ru2w: "<<endl<<Ru2w<<endl;
    cout <<"a = "<<endl<<a<<endl; 


    // below is what OKVIS's initialization process
    // align with ez_W:
    Eigen::Vector3d e_acc = a.normalized();
    Eigen::Vector3d ez_W(0.0, 0.0, -1.0);
    Eigen::Matrix<double, 6, 1> poseIncrement;
    poseIncrement.head<3>() = Eigen::Vector3d::Zero();
    poseIncrement.tail<3>() = ez_W.cross(e_acc).normalized();
    double angle = std::acos(ez_W.transpose() * e_acc);
    poseIncrement.tail<3>() *= angle;

    // T_WS.oplus(-poseIncrement);

    Eigen::Matrix<double, 6, 1> delta = poseIncrement; 
    Eigen::Vector4d dq;
    double halfnorm = 0.5 * delta.tail<3>().norm();
    dq.head<3>() = sinc(halfnorm) * 0.5 * delta.tail<3>();
    dq[3] = cos(halfnorm);
    Eigen::Quaterniond q_; 
    q_.setIdentity(); 
    cout <<"dq: "<<dq<<endl; 
    q_ = (Eigen::Quaterniond(dq) * q_);
    q_.normalize();
    // updateC();
    Eigen::Matrix<double, 3, 3> C_ = q_.toRotationMatrix();

    // cout <<"q_ = "<<endl<<q_<<endl; 
    cout <<"C_ = "<<endl<<C_<<endl; 
    Eigen::Matrix<double, 3, 3> C_inv = C_.inverse(); 
    cout <<"C_inv: "<<endl<<C_inv<<endl;
    // test 
    Eigen::Vector3d g1 = Rw2u * a; 
    Eigen::Vector3d g2 = C_ * a; 
    Eigen::Vector3d g3 = C_inv * a; 
    cout <<"g1: "<<g1<<endl; 
    cout <<"g2: "<<g2<<endl; 
    cout <<"g3: "<<g3<<endl;

    // my understanding 
    Eigen::Vector3d from_v(0, 0, -1); 
    Eigen::Vector3d to_v = a.normalized(); 
    Eigen::Vector3d w = from_v.cross(to_v).normalized(); 
    double angle_rot = acos(from_v.dot(to_v)); 
    double half_angle = angle_rot / 2.; 
    
    Eigen::Vector4d rq; 
    rq.head<3>() = w * sin(half_angle); 
    rq[3] = cos(half_angle); 
    Eigen::Matrix<double, 3, 3> R = Eigen::Quaterniond(rq).toRotationMatrix(); 
    Eigen::Matrix<double, 3, 3> R_inv = R.inverse();
    Eigen::Vector3d g4 = R_inv * a; 
    cout <<"R_inv: "<<R_inv<<endl; 
    cout <<"g4: "<<g4<<endl; 

    // 

    return 1;
}
