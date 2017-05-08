#include <iostream>
#include <sstream>
#include <iomanip>

using namespace std;

int main()
{
  stringstream ss; 
  ss<<"what "<<setfill('0')<<setw(5)<<4; 
  cout<<ss.str()<<endl;
  return 0; 
}
