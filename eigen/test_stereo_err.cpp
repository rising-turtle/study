
/*
  for stereo observation, on one image with large projection error while on the other may
  result in very small projection error 

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


int main()
{

  Vector3d pts_imu_j(0.998128, -0.129096,  0.271923);
  Vector3d tic(0.0011,  0.0217, -0.0001);
  Matrix3d qic;
  qic << -0.0282288,  0.0144013,     0.999498,
   -0.999601, -0.000418874,   -0.0282257,
   1.21787e-05,    -0.999896,    0.0144073;
   Vector3d tic2(-0.00029, -0.0579069, -0.0001919);
  Matrix3d qic2;
  qic2 << -0.0118231,   0.011553,   0.999863,
  -0.99987,  0.0108138, -0.0119481,
  -0.0109503,  -0.999875,  0.0114236;

  Vector3d l_obs(0.120773, -0.256652, 1.);
  Vector3d r_obs(0.0779299, -0.256191, 1);

  Vector3d cam_j_l = qic.transpose()*(pts_imu_j - tic);
  Vector3d cam_j_r = qic2.transpose()*(pts_imu_j - tic2);

  Vector3d un_cl = cam_j_l/cam_j_l.z();
  Vector3d un_cr = cam_j_r/cam_j_r.z();

  cout <<"un_cl: "<<un_cl.transpose()<<" l_obs: "<<l_obs.transpose()<<endl;
  cout <<"un_cr: "<<un_cr.transpose()<<" r_obs: "<<r_obs.transpose()<<endl;

  return 0;
}
