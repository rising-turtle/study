#include <vector>
#include <iostream>

using namespace std; 

class A{
public:
    A(){}
    double a;
    int b; 
};

void test1();
void test0();

int main()
{
    test1();
    return 1; 
}


void test1()
{
    vector<A> vA; 
    A a1; A a2; 
    vA.push_back(a1); 
    vA.push_back(a2);
    cout <<" vA size: "<<vA.size()<<endl; 
    vA.erase(vA.begin() + 10); 
    cout <<"vA size: "<<vA.size()<<endl;
    return ; 
}

void test0()
{
    vector<int> va{1, 2, 3, 5, 7}; 
    vector<int>::iterator ita = va.begin() + va.size(); 
    --ita; 
    while(ita != va.begin())
    {
	cout <<"*ita = "<<*ita<<" "; 
	if(*ita == 3)
	{
	    ita = va.erase(ita); 
	    cout <<endl<<"now *ita = "<<*ita<<endl; 
	}
	--ita;
    }
    cout <<endl<<"begin() : "<<*ita<<endl; 

    vector<int> va1{1}; 
    vector<int>::iterator it = va1.end();
    --it; 
    it = va1.erase(it); 
    if(it == va1.begin())
    {
	cout <<" it = va1.begin() "<<endl;
    }
    if(it == va1.end())
    {
	cout <<" it = va1.end() "<<endl;
    }

    vector<int> va2{1, 2, 3};
    cout <<"va2 size: "<<va2.size()<<endl; 
    va2.erase(va2.begin() + 2); 
    cout <<"va2 size: "<<va2.size()<<endl;

    return ;
}

