#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

const static int HALF_PATCH_SIZE = 16; // 32 

int main()
{
  const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE; 
  int v, v0, vmax = std::floor(HALF_PATCH_SIZE * sqrt(2.f)/2 + 1); 
  int vmin = std::ceil(HALF_PATCH_SIZE * sqrt(2.f)/2); 
  cout <<"vmax = "<<vmax <<" vmin = "<<vmin << endl; 

  vector<int> umax(HALF_PATCH_SIZE + 1); 
  for(v = 0; v <= vmax; ++v)
  {
    umax[v] = round(sqrt(hp2 - v*v)); 
    cout <<umax[v]<<" ";
  }
  cout << endl; 

  for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
  {
    while(umax[v0] == umax[v0+1]) 
      ++v0; 
    umax[v] = v0;
    cout <<" v = "<<v<<" umax[v] = "<<umax[v] <<" v0 = "<<v0<<endl;
    ++v0; 
  }
  for(v = 0; v< umax.size(); ++v)
  {
    cout << umax[v]<<" ";
  }
  cout<<endl;

  return 0;
}
