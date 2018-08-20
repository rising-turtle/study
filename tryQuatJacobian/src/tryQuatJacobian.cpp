/*
    try to verify the jacobians using quaternion 
    specifically, check d[Rp]/dw and d[R'p]/dw, w is the angular velocity in the tangent space of R
    check d[q1.inv() * q2]/dq1 and d[q1.inv() * q2]/dq2 
*/

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "utility.h"
#include "so3.hpp"

using namespace std; 

template<typename T>
Eigen::Quaternion<typename T::Scalar> deltaQ2(const Eigen::MatrixBase<T>& w)
{
    typedef typename T::Scalar Scalar_t; 
    Eigen::Quaternion<Scalar_t> dq; 
    Scalar_t w_norm = w.norm(); 
    Eigen::Matrix<Scalar_t, 3, 1> wm = w/w_norm; 
    Scalar_t half_w_norm = w_norm/static_cast<Scalar_t>(2.0); 
    Scalar_t cw = cos(half_w_norm); 
    Scalar_t sw = sin(half_w_norm); 
    dq.w() = cw; 
    dq.x() = wm.x() * sw; 
    dq.y() = wm.y() * sw; 
    dq.z() = wm.z() * sw;
    return dq;
}

Eigen::Quaterniond deltaQ3(Eigen::Quaterniond& Q, Eigen::Matrix<double, 3, 1>& w)
{
    Sophus::SO3 dw = Sophus::SO3::exp(w); 
    Eigen::Matrix<double, 3, 3> dw_R = Sophus::SO3::exp(w).matrix(); 
    Eigen::Matrix<double, 3, 3> R = Q.toRotationMatrix(); 
    Eigen::Matrix<double, 3, 3> R2 = R * dw_R; 
    Eigen::Quaterniond q(R2); 
    return q; 
}
std::ostream& operator<<(std::ostream& out, const Eigen::Quaterniond& q)
{
    out << q.x()<<" "<<q.y()<<" "<<q.z() <<" "<<q.w(); 
    return out; 
}

void test(); 

void test2();

void test_dR1_dw(); 

void test_dq1q2(); 

int main()
{
    // test();
    // test2();
    // test_dR1_dw(); 
    test_dq1q2(); 
    return 1;
}

void test_dq1q2()
{
    double eps= 1e-3; 
    double qx = 0.1 ; // 0.1; 
    double qy = -0.2; // -0.2; 
    double qz = 0.5; // 0.5; 
    double qw = sqrt(1 - qx*qx - qy*qy - qz*qz); 
    Eigen::Quaterniond Q1(qw, qx, qy, qz); 
    qy = 0.3; qz = -0.15; 
    qw = sqrt(1 - qx*qx - qy*qy - qz*qz); 
    Eigen::Quaterniond Q2(qw, qx, qy, qz); 
    Eigen::Quaterniond Q3 = (Q1.inverse()*Q2).normalized(); 
    cout <<"Q3 "<<Q3<<endl;
    Eigen::Matrix<double, 3, 3> num_jacobians; 

    for(int k=0; k<3; k++)
    {
	Eigen::Vector3d delta_q1 = Eigen::Vector3d(k == 0, k==1, k==2)*eps; 

	Eigen::Quaterniond Q11 = Q1 * Utility::deltaQ(delta_q1); 
	Eigen::Quaterniond Q33 = Q11.inverse() * Q2; 
	Eigen::Quaterniond delta_q3 = Q3.inverse()*Q33;
	Eigen::Vector3d delta;
	delta(0) = delta_q3.x()*2.; 
	delta(1) = delta_q3.y()*2.;
	delta(2) = delta_q3.z()*2.; 
	num_jacobians.col(k) = delta/eps; 
    }

    cout <<"numerical_jacobian: "<<endl<<num_jacobians<<endl;
    Eigen::Matrix<double, 3, 3> est_jacobians; 

    // est_jacobians = -0.5 *(Utility::Qleft(Q2.inverse()) * Utility::Qright(Q1)).bottomRightCorner<3,3>(); 
    // est_jacobians = - 0.5 * (Utility::Qleft(Q2.inverse() * Q1)).bottomRightCorner<3,3>(); 
    Eigen::Matrix<double, 3, 3> R3 =  Q3.toRotationMatrix(); 
    est_jacobians = -R3.transpose(); 
    cout<<" est_jacobian: "<<endl<<est_jacobians<<endl; 

    Eigen::Vector3d delta_q1(0.01, -0.02, 0.03);
    {
	Eigen::Quaterniond Q11 = Q1 * Utility::deltaQ(delta_q1); 
	Eigen::Quaterniond Q33 = Q11.inverse() * Q2; 
	cout <<"expected Q33: "<<endl<<Q33<<endl; 
    }
    {
	Eigen::Vector3d delta_3 = num_jacobians * delta_q1; 
	Eigen::Quaterniond Q33 = Q3 * Utility::deltaQ(delta_3); 
	cout<<"numeric jacobians Q33: "<<endl<<Q33<<endl; 
    }
    {
	Eigen::Vector3d delta_3 = est_jacobians * delta_q1; 
	// Eigen::Matrix<double ,3,3> T = ((Utility::Qleft(Q3)) * Utility::deltaQ(delta_3)).bottomRightCorner<3,3>() ;
	Eigen::Quaterniond Q33 = Q3 * Utility::deltaQ(delta_3); 
	cout<<"est jacobians Q33: "<<endl<<Q33<<endl; 
    }
	


    return ;
}

void test2()
{
    double qx = 0.1 ; // 0.1; 
    double qy = -0.2; // -0.2; 
    double qz = 0.5; // 0.5; 
    double qw = sqrt(1 - qx*qx - qy*qy - qz*qz); 
    Eigen::Quaterniond Q(qw, qx, qy, qz); 
    Eigen::Matrix3d R = Q.toRotationMatrix(); 
    Eigen::Quaterniond QQ(R); 
    Eigen::Quaterniond QQQ(QQ.toRotationMatrix()); 
    cout <<"Q: "<<Q<<endl; 
    cout <<"Q.R: "<<endl<<R<<endl;
    cout<<"QQ: "<<QQ<<endl; 
    cout <<"QQ.R: "<<endl<<QQ.toRotationMatrix()<<endl;
    cout <<"QQQ: "<<QQQ<<endl; 
    cout <<"QQQ.R: "<<endl<<QQQ.toRotationMatrix()<<endl;
}

void test_dR1_dw()
{
    double eps = 1e-3; // 1e-6 
    double qx = 0.1 ; // 0.1; 
    double qy = -0.2; // -0.2; 
    double qz = 0.5; // 0.5; 
    double qw = sqrt(1 - qx*qx - qy*qy - qz*qz); 
    Eigen::Quaterniond Q(qw, qx, qy, qz); 
    Eigen::Vector3d pi(1, 2, 3); 

    Eigen::Matrix3d R = Q.toRotationMatrix(); 

    Eigen::Vector3d pj = Q * pi; 
    cout <<"pj: "<<endl<<pj<<endl; 
    Eigen::Vector3d pj2 = R * pi; 
    cout <<"pj2: "<<endl<<pj2<<endl; 
    // Eigen::Matrix<double, 3, 3> num_jacobians; 
    Eigen::Matrix<double, 1, 3> num_jacobians; 
    double px = R.row(0) * pi; 

    for(int k=0; k<3; k++)
    {
	Eigen::Vector3d delta = Eigen::Vector3d(k == 0, k==1, k==2)*eps; 
	
	Eigen::Quaterniond Q2 = Q * Utility::deltaQ(delta); 
	// Eigen::Quaterniond Q2 = Q * deltaQ2(delta); 
	// Eigen::Quaterniond Q2 = deltaQ3(Q, delta); 

	// Eigen::Vector3d p2 = Q2 * pi;
	Eigen::Matrix3d R = Q2.toRotationMatrix(); 
	double p2 = R.row(0) * pi; 

	// Eigen::Vector3d p3 = Q2 * pi; 
	// cout <<"p2: "<<endl<<p2<<endl;
	// cout <<"p3: "<<endl<<p3<<endl; 
	// num_jacobians.col(k) = (p2 - pj)/eps; 
	num_jacobians(k) = (p2-px)/eps; 
    }
    
    cout <<"numerical_jacobian: "<<endl<<num_jacobians<<endl;

    Eigen::Matrix<double, 3, 3> jacobians = -R * Utility::skewSymmetric(pi); 
    cout <<"d[Rp]/dw = -R*[p] "<<endl<<jacobians<<endl; 
    Eigen::Matrix<double, 1, 3> R1_jacob = -R.row(0) * Utility::skewSymmetric(pi); 
    cout <<"d[R1p]/dw = -R1*[p]: "<<endl<<R1_jacob<<endl;

    return ; 

}

void test()
{
    double eps = 1e-3; // 1e-6 
    double qx = 0.1 ; // 0.1; 
    double qy = -0.2; // -0.2; 
    double qz = 0.5; // 0.5; 
    double qw = sqrt(1 - qx*qx - qy*qy - qz*qz); 
    Eigen::Quaterniond Q(qw, qx, qy, qz); 
    Eigen::Vector3d pi(1, 2, 3); 

    Eigen::Matrix3d R = Q.toRotationMatrix(); 

    Eigen::Vector3d pj = Q * pi; 
    // cout <<"pj: "<<endl<<pj<<endl; 
    Eigen::Vector3d pj2 = R * pi; 
    // cout <<"pj2: "<<endl<<pj2<<endl; 
    Eigen::Matrix<double, 3, 3> num_jacobians; 
    for(int k=0; k<3; k++)
    {
	Eigen::Vector3d delta = Eigen::Vector3d(k == 0, k==1, k==2)*eps; 
	
	Eigen::Quaterniond Q2 = Q * Utility::deltaQ(delta); 
	// Eigen::Quaterniond Q2 = Q * deltaQ2(delta); 
	// Eigen::Quaterniond Q2 = deltaQ3(Q, delta); 

	Eigen::Vector3d p2 = Q2 * pi; 
	// Eigen::Vector3d p3 = Q2 * pi; 
	// cout <<"p2: "<<endl<<p2<<endl;
	// cout <<"p3: "<<endl<<p3<<endl; 
	num_jacobians.col(k) = (p2 - pj)/eps; 
    }
    
    cout <<"numerical_jacobian: "<<endl<<num_jacobians<<endl;

    Eigen::Matrix<double, 3, 3> jacobians = -R * Utility::skewSymmetric(pi); 
    cout <<"d[Rp]/dw = -R*[p] "<<endl<<jacobians<<endl; 

    return ; 
}
