/*
 * David Z, 
 * test weather w'(uu'+vv')w = w'uu'w + w'vv'w
 * The answer is yes
 * w'uu'w = SQ(proj w^ along u^)
 * */

#include <eigen3/Eigen/Eigen>

#include <iostream>
#include <string>
#include <cmath>

using namespace std; 
using namespace Eigen;

void test1();
void test2();

#define D2R(d) ((d)*M_PI/180.)

int main()
{
  test2();
  return 0;
}

void test2()
{
  Eigen::Matrix4f R = Eigen::Matrix4f::Identity(); 
  float theta =  D2R(30); 
  float cp = cos(theta); float sp = sin(theta);  
  R(1,1) = cp;  R(1,2) = -sp; 
  R(2,1) = sp;  R(2,2) = cp; 
  Eigen::Matrix4f e = Eigen::Matrix4f::Identity(); 
   e(1,0) = 0.1; e(2,0) = 3; e(3,0) = -1;
   e(2,0) = 2.;  e(3,2) = 1.2; e(2,3) = 0.3;
  e(2,1) = 3; e(3,1) = -1; e(1,3) = -0.2;
   
  Eigen::Vector4f v1(1, 2, 3, 0); 
  Eigen::Vector4f v2 = v1;
  Eigen::Vector4f v3 = v2;
 
  v2 = e*v2; 
  cout<<"v2: "<<v2<<endl;
  v3(1) = v2(1)*cp + v2(2)*sp; 
  v3(2) = -v2(1)*sp + v2(2)*cp; 
 
  cout<<"R: "<<R<<endl;
  cout<<"R.inv: "<<R.inverse().eval()<<endl;

  Eigen::Matrix4f t1 = R.inverse().eval()*e; 
  // e = R.inverse().eval()*e; 
  cout<<"t1: "<<t1<<endl;
  v1 = t1*v1; 
 
  cout<<"v1: "<<v1<<endl;
  cout<<"v3: "<<v3<<endl;
  return ;
}

void test1()
{
  Eigen::Vector3f v(1,0,1); 
  Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
  cov = v*v.transpose();
  cout<<cov<<endl;
  cout<<v<<endl;
  Eigen::Vector3f w(0.2, 1, -0.1);
  Eigen::Vector3f u(0, 1, 0); 
  Matrix3f cov2 = u*u.transpose();
  
  double m_dis = w.transpose()*(cov+cov2)*w;
  double m_dis2 = (w.transpose()*cov*w); 
  double m_dis3 = (w.transpose()*cov2*w); 

  cout<<"m_dis: "<<m_dis<<endl;
  cout<<"m_dis2: "<<m_dis2+m_dis3<<endl;

}

