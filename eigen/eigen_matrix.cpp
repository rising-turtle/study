

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <string>
#include <cmath>

using namespace std;
using namespace Eigen;

int main()
{
    Matrix4d Ti2cl;
    Ti2cl << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
           0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
          -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
          0, 0, 0, 1;


    Matrix4d Ti2cr;
    Ti2cr << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
           0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
          -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
          0.,       0. ,          0. ,          1.;

    Matrix4d Tcr2cl;
    Tcr2cl = Ti2cr.inverse()*Ti2cl;
    cout <<"Tcr2cl: "<<endl<<Tcr2cl<<endl;

    // cout << std::fixed << endl<< A.inverse()<<endl;
    return 0;
}
