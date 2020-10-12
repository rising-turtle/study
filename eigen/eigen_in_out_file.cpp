

#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <string>
#include <cmath>
#include <iostream>
#include <fstream>

using namespace std;
using namespace Eigen;

int main()
{
    Matrix4d A;
    A << 0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
           0.999557249008, 0.0149672133247, 0.025715529948,  -0.064676986768,
           -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
           0, 0, 0, 1;

    Matrix4d B;
    B << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
           0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
          -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
          0, 0, 0, 1;
   Vector3d v(0.2, 0.1, 1.2);
   Vector3d v2(1, 2, 3.);

   ofstream ouf("out_in_file.txt");

   ouf<<A<<endl<<B<<endl;

   ouf<<v.transpose()<<" "<<v2.transpose()<<endl;

   ouf.close();

   // read it
   ifstream inf("out_in_file.txt");

   Matrix4d AA, BB;

   for(int i=0; i<4; i++)
   for(int j=0; j<4; j++){
     inf >> AA(i,j);
   }

   for(int i=0; i<4; i++)
   for(int j=0; j<4; j++){
     inf >> BB(i,j);
   }

   // inf >> AA ;
   cout<<"AA: "<< endl<<AA<<endl;
   cout<<"BB: "<< endl<<BB<<endl;

   Vector3d vv, vv2;
   for(int i=0; i<3; i++)
    inf >> vv(i);
   for(int i=0; i<3; i++)
    inf >> vv2(i);

   cout<<vv.transpose()<<" "<<vv2.transpose()<<endl;

   return 0;

}
