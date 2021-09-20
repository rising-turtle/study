// intersection between a ray and a plane 


#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <string>
#include <cmath>

using namespace std; 
using namespace Eigen; 


int main()
{
    Vector3d P1(-0.493184, 0.477806, 1.066000); 
    Vector3d P2(-0.798194, 0.8806, 2.066 ); 
    Vector3d P3(0.45, 0.11, 3.066); 


    Vector3d OP(-0.464, 0.459, 1.); 

    double x1 = P1(0); double y1 = P1(1); double z1 = P1(2); 
    double x2 = P2(0); double y2 = P2(1); double z2 = P2(2); 
    double x3 = P3(0); double y3 = P3(1); double z3 = P3(2); 
    
    double u = OP(0); double v = OP(1); 

    double s = (x1*y2*z3 - x1*y3*z2 - x2*y1*z3 + x2*y3*z1 + x3*y1*z2 - x3*y2*z1) 
	/ (x1*y2 - x2*y1 - x1*y3 + x3*y1 + x2*y3 - x3*y2 + u*y1*z2 - u*y2*z1
		- v*x1*z2 + v*x2*z1 - u*y1*z3 + u*y3*z1 + v*x1*z3 - v*x3*z1 + u*y2*z3 
		- u*y3*z2 - v*x2*z3 + v*x3*z2);
    
    cout <<"s : "<<s<<endl; 

    Vector3d P1P2 = P1 - P2; 
    Vector3d P2P3 = P2 - P3; 
    Vector3d n = P1P2.cross(P2P3); 
    cout <<"n : "<<n.transpose()<<endl; 
    n.normalize(); 
    cout <<"after normalization n: "<<n.transpose()<<endl; 
    
    s = P1.dot(n)/OP.dot(n); 
    cout <<"s: "<<s<<endl; 
    return 0;

}
