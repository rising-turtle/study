/*
 * David Z, 
 * Test weather the parameter of array will be changed after function call
 * Yes, because the input is the address of the array
 *
 * */

#include <iostream>

using namespace std; 

int assign(int a[3])
{
  a[0] = 1;
  a[1] = -1;
  a[2] = 0;
  return 0;
}

int main(int argc, char* argv[])
{
  int a[3] = {0,};
  assign(a);
  cout<<"a: "<<a[0]<<" "<<a[1]<<endl;
  return 0;
}
