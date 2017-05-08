


#include <iostream>


using namespace std; 
int main()
{

  #if defined(_WIN32)
  cout<<"Win32"<<endl;
#elif (__linux__ && (i386 || __x86_64__))
  cout<<"linux i386 || x86_64"<<endl;
#else
  cout <<"error unmatched platform"<<endl;
#endif
  cout <<"Hello "<<endl;

  return 0; 
}
