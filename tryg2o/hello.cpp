
#include <iostream>

#include "g2o/solvers/csparse/csparse_helper.h"


void func()
{
  g2o::csparse_extension::writeCs2Octave("what", NULL); 
}

int main()
{
  std::cout<<"hello world!"<<std::endl;

  return -1; 
}
