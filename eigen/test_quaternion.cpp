/*
    He Zhang, 

    test eigen quaternion normalzied 

*/


#include <eigen3/Eigen/Eigen>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

#include <iostream>

using namespace std ;


int main()
{
    Eigen::Matrix3d R; 
    R<< 0.00193013, -0.999997, 0.00115338,
      -0.999996, -0.0019327, -0.00223606,
      0.00223829, -0.00114906, -0.999997; 
    
    cout<<"R: "<<endl<<R<<endl; 

    Eigen::Quaterniond q(R); 
    cout<<"Q : " << std::endl << q.w()<<" "<<q.x()<<" "<<q.y()<<" "<<q.z()<<endl;
    R = q.normalized(); 

    cout<<"nor R: "<<endl<<R<<endl; 

    return 0; 
    
}
