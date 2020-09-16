#include <string>
#include <iostream>
#include <sstream>
// #include <chrono>

using namespace std ; 

int main()
{

    string a = "1540821358.195877075195"; 
    stringstream ss; 
    ss << a; 
    double t; 
    ss >> t; 

    cout<<std::fixed<<"t: "<<t<<endl;

    string ha = a.substr(0, 10); 
    string la = a.substr(11, 12); 
    
    cout <<"ha: "<<ha<<" la: "<<la<<endl;

    long long ht, lt; 
    {
    double ft = stod(a); 
    cout<<std::fixed << "ft: "<<ft<<endl;
    double ht = stod(ha); 
    double lt = stod(la); 
    cout <<"ht: "<<ht<<" lt: "<<lt<<endl;
    }
    stringstream ssha, ssla; 
    ssha << ha; 
    ssha >> ht; 
    ssla << la; 
    ssla >> lt; 

    cout <<std::fixed<<" ht: "<<ht<<" lt: "<<lt<<endl; 
    double ft = ht + lt*1./1e12;
    cout <<std::fixed<<ft<<endl;

    long long tt = ht * 1e9 + lt*1e-3;
    cout <<tt*1e-9<<endl;


    return 0; 

}
