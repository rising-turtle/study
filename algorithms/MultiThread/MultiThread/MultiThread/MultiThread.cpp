#include "preheader.h"
#include <windows.h>
#include <process.h>
namespace{
// _beginthread(void(*)(), int stack, void* param )
// _endthread()
ofstream outf("outf.txt");

void thread1(PVOID PARAM){
	while(1){
		Sleep(20);
		int * param=(int*)(PARAM);
		cout<<"input "<<*param<<endl;
		outf<<"This is thread1: "<<*param<<endl;
	}
}

void thread2(PVOID PARAM){
	while(1){
		Sleep(20);
		char* param = (char*)PARAM;
		cout<<"input "<<*param<<endl;
		outf<<"This is thread2: "<<*param<<endl;
	}
}

};
void main1(){
	int tp1=1;
	char tp2='t';
	_beginthread(thread1,0,&tp1);
	_beginthread(thread2,0,&tp2);
	Sleep(1000);
	cout<<"input 'e' or 'E' to exit!"<<endl;
	char inputc;
	while(1){	
		inputc=getchar();
		if(inputc=='e' || inputc=='E'){
			cout<<"exit!"<<endl;
			break;
		}
	}
	outf.close();
	return ;
}