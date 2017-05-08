#include <vector>
#include <iostream>
using namespace std; 


int main(int argc, char* argv[])
{
  int N = 10; 
  vector<bool> vb1(N, false); 
  vector<bool> vb2(N, false); 
  vector<bool> vb3(N, false);
  vb3[N-2] = true; 
  if(vb1 == vb2) cout<<"vb1 == vb2"<<endl;
  else cout<<"vb1!=vb2"<<endl;

  if(vb1 == vb3) cout<<"vb1 == vb3"<<endl;
  else cout<<"vb1 != vb3"<<endl;

  return 1;
}
