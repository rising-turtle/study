#include <cmath>
#include <iostream>

using namespace std;
#define R2D(r) ((r)*180./M_PI)

int main(int argc, char* argv[])
{
  float y = 0.86453; 
  float z = -0.498; 

  float a = atan2(z, y); 
  cout<<"atan2(z,y) = "<<R2D(a)<<endl;
  return 0;
}
