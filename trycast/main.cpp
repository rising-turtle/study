#include <iostream>

using namespace std;

class A{
public:
	int c;
	int a;
};

class B : public A{
public:
	int b;
};

void fun(void* p)
{
	A* pa = static_cast<A*>(p);
	if(pa == NULL){
		cout<<"p is not type of A!"<<endl;
	}
	else{
		cout<<"p is type of A: "<<pa->a<<endl;
	}
	return ;
}

int main()
{
	B b;
	b.a = -1;
	fun((void*)(&b));
	int a = 1;
	fun((void*)(&a));
	fun((void*)(NULL));
	return 0;
}
