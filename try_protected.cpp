#include <iostream>
using namespace std;

class B
{
  protected:
    void sthProtected()
    {
      cout<<"B::sthProtected()"<<endl;
    }
};

class D : public B
{
  public:
    void sthPublic()
    {
      cout<<"D::sthProtected()"<<endl; 
      sthProtected();
    }
};

int main()
{
  D p; 
  p.sthPublic();
}



