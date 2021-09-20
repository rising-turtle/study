/*
  10.12.2020 He Zhang, fuyinzh@gmail.com
  some rotation matrix cannot hold for itself, which is the reason why
  VINS-Stereo diverges when adding stereo constraint, since the Tic2
  is not self consistent
*/



#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <string>
#include <cmath>

using namespace std;
using namespace Eigen;

int main()
{
    Matrix3d R;
    // This is Tic1 in FPV calibration file
    R << -0.02822879, 0.01440125,  0.99949774,
          -0.99960149, -0.00041887, -0.02822568,
          0.00001218, -0.99989621,  0.01440734;

    // This is Tic2 in Euroc calibration file
    R << 0.0125552670891, -0.999755099723, 0.0182237714554,
           0.999598781151, 0.0130119051815, 0.0251588363115,
          -0.0253898008918, 0.0179005838253, 0.999517347078;

    // This is Tic2 in TUM VI calibration file
    R << -0.999511,  0.00810408,  -0.0301991,
          0.0302991,   0.0125116,   -0.999463,
          -0.00772188,   -0.999889,  -0.0127511;
    // This is Tic2 in FPV calibration file
    R << -0.01182306, 0.01155299, 0.99986336,
            -0.99987014,  0.01081377, -0.01194809,
            -0.01095033 , -0.99987479,  0.01142364;
    Matrix3d Rt = R.transpose();

    Matrix3d RRt = R*Rt;
    cout <<"RRt: "<<endl<<RRt<<endl; 

    Quaterniond q{R};

    cout<<"before "<<" R: "<<endl<<R<<endl;
    cout<<"q: "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;

    for(int r= 1; r<100; r++){

      R = q.toRotationMatrix();
      q = Quaterniond(R);
    }

    cout<<"after 100 rounds "<<" R: "<<endl<<R<<endl;
    cout<<"q: "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;

    return 0;
}
