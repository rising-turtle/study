/*
 * David Z, 
 * test the map scheme in Eigen
 * */

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

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
  // test1();
  test2();
  return 0;
}


void test2()
{
  double buf[8]; 
  Map<Matrix<double, 2, 4, Eigen::RowMajor> > Mr(buf); 
  Map<Matrix<double, 2, 4, Eigen::ColMajor> > Mc(buf); 
  Map<Matrix<double, 4, 2, Eigen::ColMajor> > Mct(buf); 
  Map<Matrix<double, 4, 2, Eigen::RowMajor> > Mrt(buf);
  Matrix2d t1; 
  Matrix2d t2; 
  t1 << 1, 2, 3, 4; 
  t2 << -1, -2, -3, -4; 
  Map<Matrix2d> Mt1(buf); Map<Matrix2d> Mt2(buf+4); 
  Mt1 = t1; Mt2 = t2; 
  cout<<"RowMajor Mr: "<<endl<<Mr<<endl;
  cout<<"ColMajor Mc: "<<endl<<Mc<<endl;
  cout<<"RowMajor Mrt: "<<endl<<Mrt<<endl;
  cout<<"ColMajor Mct: "<<endl<<Mct<<endl;
}

template <typename Derived, typename DerivedOther>
void skew(Eigen::MatrixBase<Derived>& Sx, 
    Eigen::MatrixBase<Derived>& Sy, 
    Eigen::MatrixBase<Derived>& Sz, 
    const Eigen::MatrixBase<DerivedOther>& R){
  const double 
  r11=2*R(0,0), r12=2*R(0,1), r13=2*R(0,2),
  r21=2*R(1,0), r22=2*R(1,1), r23=2*R(1,2),
  r31=2*R(2,0), r32=2*R(2,1), r33=2*R(2,2);
  Sx <<    0,    0,    0,  -r31, -r32, -r33,   r21,   r22,  r23;
  Sy <<  r31,  r32,  r33,     0,    0,    0,  -r11,  -r12, -r13;
  Sz << -r21, -r22, -r23,   r11,   r12, r13,     0,    0,    0;
}


void test1()
{
  double buf[27]; 
  Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::ColMajor> > M(buf); 
  Matrix3d Sxt, Syt, Szt; 
  // Matrix<double, 3, 3, Eigen::RowMajor> Sxt, Syt, Szt;
  typedef  Matrix<double, 3, 3, Eigen::RowMajor> Matrix3DR; 
  Matrix3d Rb; 
  Rb << 1 , 2 , 3, 
        4 , 5, 6,
        7, 8, 9; 
  skew(Sxt, Syt, Szt, Rb); 
  Eigen::Map<Matrix3DR > Mx(buf); 
  Eigen::Map<Matrix3d> My(buf+9); 
  Eigen::Map<Matrix3d> Mz(buf+18); 
  Mx = Sxt; My = Syt; Mz = Szt; 
  
  cout<<"matrix Rb: "<<endl<<Rb<<endl;
  cout<<"matrix Sxt: "<<endl<<Sxt<<endl;
  cout<<"matrix Syt: "<<endl<<Syt<<endl;
  cout<<"matrix Szt: "<<endl<<Szt<<endl;
  cout<<"matrix Mx: "<<endl<<Mx<<endl;
  cout<<"matrix My: "<<endl<<My<<endl;
  cout<<"matrix Mz: "<<endl<<Mz<<endl;
  cout<<"matrix M: "<<endl<<M<<endl;

  return ;
}



