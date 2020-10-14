

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <string>
#include <cmath>

using namespace std;
using namespace Eigen;

int main()
{
    Matrix4d A;
    A <<  -0.02822879, 0.01440125,  0.99949774, 0.0011,
          -0.99960149, -0.00041887, -0.02822568, 0.0217,
          0.00001218, -0.99989621,  0.01440734, -0.0001,
          0, 0, 0, 1;


    Matrix4d B;
    B << -0.01182306, 0.01155299, 0.99986336, -0.00029,
      -0.99987014,  0.01081377, -0.01194809,  -0.0579069465,
      -0.01095033 , -0.99987479,  0.01142364, -0.0001919,
      0,                   0,                   0,   1.0;

    Matrix4d C;
    C = B.inverse()*A;
    cout <<"C: "<<endl<<C<<endl;

    Quaterniond q1( 0.502625, -0.491383, 0.502767, -0.50307 );
    Quaterniond q2( -0.502625, 0.491383, -0.502767, 0.50307);

    cout<<"R1: "<<endl<<q1.toRotationMatrix()<<endl;
    cout<<"R2: "<<endl<<q2.toRotationMatrix()<<endl;

    Matrix3d R;
    R << -0.01182306, 0.01155299, 0.99986336,
      -0.99987014,  0.01081377, -0.01194809,
      -0.01095033 , -0.99987479,  0.01142364;

    Quaterniond q{R};
    for(int r= 1; r<100; r++){
      cout<<"round "<<r<<" R: "<<endl<<R<<endl;
      cout<<"q: "<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<endl;
      R = q.toRotationMatrix();
      q = Quaterniond(R);
    }


    // cout << std::fixed << endl<< A.inverse()<<endl;
    return 0;
}
