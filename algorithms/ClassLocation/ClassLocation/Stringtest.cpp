
#include <iostream>
#include <malloc.h>
#include "String1.h"

using namespace std;


void getchars(const char* a3){
	cout<<" a3 "<<sizeof(a3)<<endl;
}

void testString()
{
	char a1[100];
	char * a2 = new char[100];
	cout<<" a1 "<<sizeof(a1)<<endl;
	cout<<" a2 "<<sizeof(a2)<<endl;
	getchars(a2);
}
//void main(){
//	//testString();
//	String1 s1("haha");
//	String1 s2(s1);
//	String1 s3 = s2;
//	cout<<s1<<s2<<s3;
//	
//	getchar();
//};