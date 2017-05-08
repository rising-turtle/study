#include "TestSet.h"
#include "preheader.h"
#include "stdio.h"
#include "CTemplate.h"

void testPointer()
{
	int a[10]={0,1,2,3,4,5,6,7,8,9};
	int (*p)[10];
	int (**pp)[10];

	p=&a;
	pp=&p;

	cout<<"Array a: "<<hex<<a<<endl;
	cout<<"P of &a: "<<hex<<p<<", *p:"<<(*p)<<endl;
	cout<<"PP of &a: "<<hex<<pp<<endl;

	cout<<"&p: "<<&p<<", &pp: "<<&pp<<endl;
	
	cout<<"a[0]: "<<a[0]<<", *p: "<<(*p)[0]<<", *pp: "<<(*pp)[0]<<", **pp: "<<(**pp)[0]<<endl;
	cout<<"p+1: "<<(p+1)<<", *(p+1): "<<(*(p+1))<<endl;
	cout<<"*pp+1:"<<(*(pp)+4)<<", *(*(pp)+1):"<<(*(*(pp)+4))<<endl;
	cout<<"pp+1: "<<(pp+4)<<", **(pp+1):"<<(**(pp+4))<<endl;
}

class A{};
class B : virtual public A{};
class C : virtual public A{};
class D : public B, public C{};

typedef A lll;
class CP{
private:
	typedef double lll;
	lll _val1;
public:
	CP(lll val){_val1=val;}
	~CP(){}
	lll getV(){return _val1;}

};

void testClassSizeOf()
{
/*	cout<<"sizeof(A):"<<sizeof(A)<<endl;
	cout<<"sizeof(B):"<<sizeof(B)<<endl;
	cout<<"sizeof(C):"<<sizeof(C)<<endl;
	cout<<"sizeof(D):"<<sizeof(D)<<endl;*/
	int a=1;
	double b=2;
	CP p(a);
	cout<<"Val:"<<p.getV()<<endl;
}

class VA{
public:
	VA(){}
	~VA(){}
	void display(){}
public:
	float x,y,z;
};
class VB: virtual public VA{
	virtual void myvirfun(){}
};

void testVPTR()
{
	VA va1;
	//VB vb1;
	cout<<"sizeof(VA): "<<sizeof(VA)<<endl;
	cout<<"sizeof(VB): "<<sizeof(VB)<<endl;
	printf("offset of VA::z =%p\n",&VA::z);
	printf("offset of va1.z=%p\n",(char*)&va1.z-(char*)&va1);
	void* Nu=NULL;
	// 类成员指针如果没有指向真实的 Member::Data，就表示它所指向数据成员在类中的Offset，
	float VA::*p1=0;
	float VA::*p2=&VA::z;
	printf("Nu=%p\n",Nu);
	printf("p1=%p\n",p1);
	printf("p2=%p\n",p2);
	// 区分指向真实的Member::Data 和没有指向真实数据的类成员指针
	// 有的编译器是+1操作，使得所有指向真实的Member::Data的指针值+1，
	// 不同的编译器有不同的区分方法。
	// 这种方式存取数据效率低
	printf("va1.*p2=%p\n",va1.*p2);
}

class Vptr{
public:
	virtual ~Vptr(){}
	virtual void toa(){}
};
class Vptr1 {
public:
	virtual ~Vptr1(){}
};
class V2 : public Vptr, public Vptr1{
public:
	virtual void derived(){}
};

void testVPTR1()
{
	//Vptr a;
	//V2 b;
	//Vptr& ra=a;
	//Vptr* pa=new V2;
	//cout<<"a: "<<sizeof(a)<<endl;
	//cout<<"b: "<<sizeof(b)<<endl;
	//cout<<"ra: "<<sizeof(ra)<<endl;
	//cout<<"pa: "<<sizeof(*pa)<<endl;
	//delete pa;
	V2 v;
	cout<<"sizeof(V2): "<<sizeof(v)<<endl;
	Vptr* pv1=new V2;
	Vptr1* pv2=new V2;
	cout<<"sizeof(*V1): "<<sizeof(*pv1)<<endl;
	cout<<"sizeof(*V2): "<<sizeof(*pv2)<<endl;
	delete pv1;
	delete pv2;
}


void testSet()
{
	int a[1];
	int *pa=a;
	int (*pa1)[1]=&a;
	cout<<hex;
	cout<<"a: "<<a<<", pa: "<<pa<<", *pa: "<<*pa<<endl;
	cout<<"&a: "<<&a<<", pa1: "<<pa1<<", *pa1"<<pa1<<endl;
}

class CA{
public:
	CA(int a=1){}
	CA(const CA& ){
		cout<<"CA Copy Constructor!"<<endl;
	}
};

class CB/*: public CA*/{
public:
	CA ca;
	CB(){}
	//CB(const CB&){
	//	cout<<"CB Copy Constructor!"<<endl;
	//}
	//CB& operator=(const CB&){
	//	cout<<"CB Assignment !"<<endl;
	//	return (*this);
	//}
};

void testConstruct()
{
	CB cb;
	CB cb1=cb;
	cb=cb1;
}

double foo(double a)
{
	cout<<"foo(double a): "<<a<<endl;
	return a;
}


void testTemplate()
{
	CTemplate1<int> ta;
	ta.type_Independent();
	ta.type_dependent();
}

// call dynamic_cast 向下转型，必须针对 实现多态的类，即含有vtpr，因为vtpr->[0]有类型info 
// call static_cast 只是内存上的强制转型，可能存在危险，
class D1{
public:
	D1():a1(1){}
	virtual ~D1(){
		cout<<"~D1 called!"<<endl;
	}
	virtual void display(){cout<<"D1: a1="<<a1<<endl;}
	int a1;
};


class D2:public D1{
public:
	D2():a2(2){}
	~D2(){
		cout<<"~D2 called!"<<endl;
	}
	void display(){cout<<"D2: a2="<<a2<<endl;}
	int a2;
};

void dynamic_castCC()
{
	D1* pd=new D2;
	//D2* pd2=dynamic_cast<D2*>(pd);
	D2* pd2=static_cast<D2*>(pd);
	pd->display();
	pd->D1::display();
	if(pd2==0)
	{
		cout<<"error in dynamic_cast!"<<endl;
	}
	else{
		pd2->display();
		cout<<"succeed in dynamic_cast!"<<endl;
	}
	delete pd;
}