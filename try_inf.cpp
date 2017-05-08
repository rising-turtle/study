

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip>

using namespace std; 


int main()
{
  // ifstream inf("long_double.txt"); 
  string s("1491332638.328436731");
  // getline(inf, s); 
  stringstream ss; 
  ss << s; 
  cout <<"ss = "<<ss.str()<<endl;
  double t; 
  ss >> std::setprecision(9) >>t; 
  
  cout << std::fixed<< std::setprecision(9) << t<<endl; 

  printf("read line %s \n", ss.str().c_str()); 
  printf("t = %.9lf\n", t);
  
  double t2 = atof(s.c_str());
  printf("t2 = %.9lf\n", t2);

  double t3 = strtold(s.c_str(), 0); 
  printf("t3 = %.9lf\n", t3);

  double t4 = 1491332638.328436731; 
  printf("t4 = %.9lf\n", t4);

  double t5 = 100000000.328436731; // 8.000001; // 0.6257381; 
  printf("t5 = %.9lf\n", t5);
    return 0; 
}
