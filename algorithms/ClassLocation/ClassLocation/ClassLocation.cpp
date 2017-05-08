#include <iostream>
using namespace std;

class A{
public:
	int a;
};

class B : virtual  public A{
public:
	int b;
};
class C : virtual  public A{
public:
	 int c;
};
class D : public C, public B{
public:
	 int d;
};

void test1(){
	B pb;
	D pd;
	pb.b = 2;
	pb.a = 1;
	int a = 0 , b = 0, c =0;
	memcpy(&a,(int*)&pb,4);
	memcpy(&b,(int*)&pb+1,4);
	memcpy(&c,(int*)&pb+2,4);

	cout<<hex<<(int*)&pd<<"  &pd "<<endl;
	cout<<hex<<(int*)&pd.d<<" &pd.d"<<endl;
	cout<<hex<<(int*)(C*)&pd<<" (C*)&pd "<<endl;
	cout<<hex<<(int*)&pd.c<<" &pd.c "<<endl;
	cout<<hex<<(int*)(B*)&pd<<"  (B*)&pd "<<endl;
	cout<<hex<<(int*)&pd.b<<" &pd.b "<<endl;
	cout<<hex<<(int*)&pd.a<<" &pd.a  "<<endl;
	cout<<hex<<(int*)(A*)&pd<<" (A*)&pd "<<endl;
	//cout<<a<<" "<<b<<" "<<c<<endl;
}

class BB{};//int bb;};
class CC{int cc;};
class DD : public BB, public CC{};

void test2(){
	DD d;
	cout<<hex<<&d<<" &d "<<endl;
	cout<<hex<<(BB*)&d<<" (BB*)&d"<<endl;
	cout<<hex<<(CC*)&d<<" (CC*)&d"<<endl;
}

class AAA{public:int a;};
class BBB{public:int b;};
class CCC : public AAA{};
class DDD : public BBB{};
class EEE : public CCC, public DDD{};
void test3(){
	EEE e;
	cout<<hex<<&e<<endl;
	cout<<hex<<(CCC*)&e<<" CCC"<<endl;
	cout<<hex<<(DDD*)&e<<" DDD"<<endl;
	cout<<hex<<(BBB*)&e<<" BBB"<<endl;
	cout<<hex<<(AAA*)&e<<" AAA"<<endl;
	cout<<hex<<&e.a<<endl;
	cout<<hex<<&e.b<<endl;
}

class A1{};
class B1 : virtual public A1{};
class C1 : virtual public A1{ };
class D1 : virtual public B1{};
class E1 : virtual public C1{};
class F1 : public E1, public D1{};

void test4(){
	F1 f;
	cout<<hex<<&f<<endl;
	cout<<hex<<(E1*)&f<<endl;
	cout<<hex<<(D1*)&f<<endl;
	cout<<hex<<(C1*)&f<<endl;
	cout<<hex<<(B1*)&f<<endl;
	cout<<hex<<(A1*)&f<<endl;
}

void test5(const A1& t){}

//void main(){
//	
//	//test1();
//	//test2();
////	BBB b;
////	cout<<sizeof(b)<<endl;
//	//test3(); 
//	//test4();
//	A1 a;
//	A1 *pa = &a;
//	test5(*pa);
//	getchar();
//};