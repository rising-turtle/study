
/*
test the stereo correction
*/


#include <eigen3/Eigen/Eigen>

#include <stdio.h>
#include <iostream>
#include <string>
#include <cmath>
#include <fstream>
#include <vector>

using namespace std;
using namespace Eigen;

int COL = 640;
int ROW = 480;
float FOCAL = 460;

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

void calTriangulateError(Eigen::Matrix<double, 3, 4> &leftPose, Eigen::Matrix<double, 3, 4> &rightPose,
                        Eigen::Vector2d &point0, Eigen::Vector2d &point1, Eigen::Vector3d &pt3d)
{
  triangulatePoint(leftPose, rightPose, point0, point1, pt3d);
  Eigen::Matrix3d Rrl = rightPose.leftCols<3>();
  Vector3d trl = rightPose.rightCols<1>();
  Vector3d pt3dj = Rrl * pt3d + trl;

  cout <<"pt3d: "<<pt3d.transpose()<<" depth: "<<pt3d.z()<<endl;

  Vector2d pp0(pt3d.x()/pt3d.z(), pt3d.y()/pt3d.z());
  Vector2d pp1(pt3dj.x()/pt3dj.z(), pt3dj.y()/pt3dj.z());

  cout <<" on left image x: "<<point0.transpose()<<" proj(x): "<<pp0.transpose()<<endl;
  cout <<" on right image x: "<<point1.transpose() << " proj(x): "<<pp1.transpose()<<endl;
  // left/right projective error
  Vector2d lpe = pp0 - point0;
  Vector2d rpe = pp1 - point1;
  cout <<"lpe: "<<lpe.transpose()<<" lpe_norm: "<<lpe.norm()<<endl;
  cout <<"rpe: "<<rpe.transpose()<<" rpe_norm: "<<rpe.norm()<<endl;
}


struct pt_pair{
    float xi, yi, xj, yj;
};

bool readfile( string fname, std::vector<pt_pair>& v);

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
        q(2), typename Derived::Scalar(0), -q(0),
        -q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

int main()
{
    string fname = "left-right.txt";
    vector<pt_pair> v;
    if(!readfile(fname, v))
      return - 1;

      Eigen::Matrix<double, 3, 4> leftPose = Eigen::Matrix<double, 3, 4>::Zero();
      leftPose.leftCols<3>() = Eigen::Matrix3d::Identity();

      Eigen::Matrix<double, 3, 4> rightPose = Eigen::Matrix<double, 3, 4>::Zero();
      rightPose.leftCols<3>() = Eigen::Matrix3d::Identity();

      Eigen::Matrix3d Rrl;
      Rrl<< 0.9998053, 0.01119774, 0.01624713,
            -0.01114776, 0.99993286, -0.00316357,
            -0.01628147, 0.00298183, 0.999863;
      Eigen::Vector3d trl = Eigen::Vector3d(-0.07961594, 0, 0);
      rightPose.leftCols<3>() = Rrl;

      rightPose.rightCols<1>() = trl; // 0.00074435, 0.00044255);
      Eigen::Matrix3d Rlr = Rrl.transpose();

      // traditional expression as in wikipidia
      // Vector3d t = tlr;
      // Eigen::Matrix3d R = Rrl;

      // same as the book "Statistical Optimization for Geometric Computation "
      Vector3d h = trl;
      Eigen::Matrix3d R = Rlr;
      Eigen::Matrix3d SH = skewSymmetric(h);
      Eigen::Matrix3d G = SH*R;
      Eigen::Matrix3d Gt = G.transpose();

    for(int i=0; i<v.size() && i < 3; i++){
    //for(int i=0; i<1; i++){
      Vector2d point0((v[i].xi-COL/2)/FOCAL, (v[i].yi-ROW/2)/FOCAL);
      Vector2d point1((v[i].xj-COL/2)/FOCAL, (v[i].yj-ROW/2)/FOCAL);
      Vector3d pt3d;

      calTriangulateError(leftPose, rightPose, point0, point1, pt3d);

      // epipolar error
      Vector3d np0(point0.x(), point0.y(), 1.);
      Vector3d np1(point1.x(), point1.y(), 1.);
      double fe = np0.transpose() * G * np1;
      double de1 = np1.transpose() * Gt * G * np1; double de2 = (np0.transpose() * G * Gt * np0);
      double de = de1 + de2;
      Vector3d dnp0 = fe * G * np1;
      Vector3d dnp1 = fe * Gt * np0;

      Vector3d new_np0 = np0 - dnp0/de;
      Vector3d new_np1 = np1 - dnp1/de;
      cout <<"fe: "<<fe<<endl;
      double new_fe = new_np0.transpose() * G * new_np1;
      cout <<"new fe: "<<new_fe<<endl;

      Vector2d new_point0(new_np0.x(), new_np0.y());
      Vector2d new_point1(new_np1.x(), new_np1.y());

      cout <<" updated projective model: "<<endl;
      calTriangulateError(leftPose, rightPose, new_point0, new_point1, pt3d);


    }

    return 0 ;

}


bool readfile( string fname, std::vector<pt_pair>& v){

  ifstream inf(fname.c_str());
  char buf[1024];
  pt_pair pt;
  while(inf.getline(buf, 1024)){
    sscanf(buf, "%f %f %f %f", &pt.xi, &pt.yi, &pt.xj, &pt.yj);
    v.push_back(pt);
  }

  cout <<"succeed to get "<<v.size()<<" feature points!"<<endl;
  return v.size() > 0;
}
