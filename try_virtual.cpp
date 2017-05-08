/*
 * David Z, 
 * test the sequence of the calling of the virtual function 
 * */

#include <iostream>

using namespace std;

class Base
{
public:
  Base(){}
  virtual ~Base()
  {
    cout<<"Base::~Base()"<<endl;
  }
  virtual void go()
  {
    start();
    add();
    self();
  }
  virtual void start() = 0 ;
  virtual void add() = 0; 
  virtual void self()
  {
    cout<<"Base::self()"<<endl;
    self_A();
  }
  virtual void self_A()
  {
    cout<<"Base::self_A()"<<endl;
  }
};

class C1 : public Base
{
public:
  C1(){}
  virtual ~C1()
  {
    cout<<"C1::~C1()"<<endl;
  }
  virtual void start()
  {
    cout<<"C1::start()"<<endl;
    return ;
  }
  virtual void add()
  {
    cout<<"C1::add()"<<endl;
    return; 
  }
  // virtual void self()
  // {
  //  cout<<"C1::self()"<<endl;
  //  return ;
  // }
  virtual void self_A()
  {
    cout<<"C1::self_A()"<<endl;
  }
};

class C11 : public C1
{
public:
  C11(){}
  virtual ~C11()
  {
    cout<<"C11::~C11()"<<endl;
  }
  virtual void start()
  {
    cout<<"C11::start()"<<endl;
    return ;
  }
};

// void let_us_go(Base*);

int main(int argc, char* argv[])
{
  Base* pb = new C11; 
  pb->go();
  delete pb;
  
  C1 c1; 
  c1.Base::self();
  
  return 0;
}









