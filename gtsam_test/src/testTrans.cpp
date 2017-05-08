#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/LieVector.h>
#include <cmath>
#include <iostream>

using namespace std;
using namespace gtsam;

#define D2R(d) (((d)*M_PI)/180.)

void test1(); 
void test2();   // use IMU as the center frame 
void test3();   // use camera as the center frame  

int main()
{
  // test1();
  // test2();
  // test3(); 
  
  // Rot3 r = Rot3::RzRyRx(D2R(-112.4), D2R(-21.46), D2R(-1.338)); 
  
  double g = 9.7; 
  Vector3 m; m << -3.31 , 1.22, -9.055; 
  Rot3 r = Rot3::RzRyRx(D2R(-1.338), D2R(-21.46), D2R(-112.4)); 
  double bg = -1, be = -1; 
  
  while(g <= 10)
  {
    Vector3 gravity; gravity << 0, 0, g;
    Vector3 measuredAcc = r.unrotate(-Point3(gravity)).vector();
    Vector3 ev =  (measuredAcc - m); 
    double e = ev.norm(); 
    cout <<" g = "<<g<<" e = "<<e<<endl;  
    if(be <0 || be > e)
    {
      be = e; 
      bg = g; 
    }

    g += 0.01; 
    //  cout <<"measured gravity : "<<measuredAcc<<endl; 
    //
  }
  cout <<"best bg = "<<bg<<" be = "<<be<<endl; 
  Vector3 gravity; gravity << 0, 0, bg;
  Vector3 measuredAcc = r.unrotate(-Point3(gravity)).vector();
  cout <<"measured gravity:\n "<<measuredAcc<<endl;


  return 0; 
}

void test3()
{
  Rot3 RI; 
  Point3 tI; 
  Point3 t(0, 0, 1); 
  Rot3 R1 = Rot3::Rx(-30*M_PI/180); 
  // t = R1.inverse() * t; 
  // Rot3 Rc2u = R1.inverse() *  Rot3::Rx(90*M_PI/180) *  Rot3::Rz(M_PI) ;
  Rot3 Ru2c =  Rot3::Rz(M_PI) * Rot3::Rx(-90*M_PI/180) * R1; 
  // Pose3 cTu(R1.inverse(), t);
  Pose3 uTc(Ru2c, t);

  Point3 tu12(0, -0.5, 0.133975);
  Pose3 u12(R1, tu12); 
  Pose3 u1; 
  Pose3 c1; 
  Pose3 c12 = uTc.inverse() * u12 * uTc; 
  Pose3 c2 = c1*c12 ; 
  Pose3 u2 = u1 * u12;
  cout << "u1: "<<endl<<u1<<endl;
  cout << "u12: "<<endl<<u12<<endl; 
  cout <<"u2: "<<endl<<u2<<endl;
  cout <<"uTc: "<<endl<<uTc<<endl;
  cout <<"c1: "<<endl<<c1<<endl;
  cout <<"c2: "<<endl<<c2<<endl;
 
}

void test2()
{
  Rot3 RI; 
  Point3 tI; 
  Point3 t(0, 1, 0); 
  Rot3 R1 = Rot3::Rx(-30*M_PI/180); 
  t = R1.inverse() * t; 
  Pose3 c1(R1, tI); 
  Pose3 c12(R1.inverse(), tI); 
  // Rot3 Rc2u = Rot3::Rz(M_PI) * Rot3::Rx(90*M_PI/180)* R1.inverse();
  Rot3 Rc2u =  R1.inverse() *  Rot3::Rx(90*M_PI/180) *  Rot3::Rz(M_PI) ;

  // Pose3 cTu(R1.inverse(), t);
  Pose3 cTu(Rc2u, t);

  Pose3 u1; 
  Pose3 c2 = c1*c12 ; 
  Pose3 u12 = cTu.inverse() * c12 * cTu; 
  Pose3 u2 = u1 * u12;
  cout <<"c1: "<<endl<<c1<<endl;
  cout <<"c2: "<<endl<<c2<<endl;
  cout <<"cTu: "<<endl<<cTu<<endl;
  cout << "u1: "<<endl<<u1<<endl;
  cout << "u12: "<<endl<<u12<<endl; 
  cout <<"u2: "<<endl<<u2<<endl;
}

void test1()
{
  Rot3 RI; 
  Point3 tI; 
  Point3 t(0, 0, -1); 
  Rot3 R1 = Rot3::Ry(90*M_PI/180); 
  Pose3 c1; 
  Pose3 c12(R1, tI); 
  Pose3 cTu(RI, t);
  Pose3 u1; 
  Pose3 c2 = c1*c12 ; 
  Pose3 u12 = cTu.inverse() * c12 * cTu; 
  Pose3 u2 = u1 * u12;
  cout <<"c2: "<<endl<<c2<<endl;
  cout << "u12: "<<endl<<u12<<endl; 
  cout <<"u2: "<<endl<<u2<<endl;

}






