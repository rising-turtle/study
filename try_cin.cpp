#include <iostream>
#include <fstream>

/**
 * cin >> variable, get value according to the type of the variable, and 
 * it will ignore the first "space" "enter" "tab", stop at the second "space" "enter" "tab"
 * */

using namespace std; 

int main(int argc, char* argv[])
{
  ofstream ouf("tmp.log"); 
  ouf<<"ab    "<<endl<<"  cd "<<endl;
  ouf<<1<<endl;
  ouf<<3.14;

  ouf.close(); 

  ifstream inf("tmp.log"); 
  string s1, s2; 
  int b1; 
  float f1; 
  inf>>s1; 
  inf>>s2;
  inf>>b1; 
  inf>>f1;
  cout<<"s1: "<<s1<<"s2: "<<s2<<" b1: "<<b1<<" f1: "<<f1<<endl; 
  inf.close();


  return 0; 
}
