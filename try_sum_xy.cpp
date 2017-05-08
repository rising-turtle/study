#include <iostream>

using namespace std;



int main()
{
  int s = 0;
  for(int u=-5; u<=5; u++)
  {
    for(int v = -2; v<=2; v++)
    {
      s += u*v; 
    }
  }

  cout<<"s= "<<s<<endl;
}
