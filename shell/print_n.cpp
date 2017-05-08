#include <iostream>
#include <stdlib.h>

using namespace std; 
int main(int argc, char* argv[])
{
  int n = -1;
  if(argc >=2)
    n = atoi(argv[1]);
  cout<<"print_"<<n<<" "<<endl;
  cin>>n;
  return 0; 
}
