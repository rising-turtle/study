#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>

using namespace std;

int main(int argc, char* argv[])
{
  int n = -1;
  if(argc >= 2)
  {
    n = atoi(argv[1]);
  }
  stringstream ss;
  ss<<"write_"<<n<<".log";
  ofstream ouf(ss.str().c_str());
  ouf<<n<<endl;
  return 0;
}
