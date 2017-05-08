#include <iostream>

using namespace std; 


class A
{
  public:
    A(){}
    virtual ~A(){}
    virtual void print_c(){cout<<"A::print_c()"<<endl;}
    virtual void try_to_print(){print_c();}
};

class B : public A
{
  public:
    B(){}
    virtual ~B(){}
    virtual void try_to_print(){ //A::try_to_print();
      print_c();
    }
};

int main()
{
  A* pa = new B; 
  ((B*)pa)->try_to_print();
  return 0; 
}
