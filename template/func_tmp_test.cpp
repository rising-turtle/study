#include "func_temp.h"

int main()
{
  int a = 2; 
  float b = 3.1415; 
  
  float * pa = static_cast<float*>(&a);

  // swap(&a, &b); 
  // float* pca = cast_go<int, float>(&a);
  // int * pia = cast_go<float, int>(pca);
  return 0;
}
