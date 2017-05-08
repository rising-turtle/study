#include "preheader.h"
#include <Windows.h>
#include <process.h>
namespace{
	UINT threadf1(LPVOID PARAM){
		while(1){
			Sleep(20);
			cout<<"This is thread1"<<endl;
		}
		return 1;
	}
	UINT threadf2(LPVOID PARAM){
		while(1){
			Sleep(20);
			cout<<"This is thread2"<<endl;
		}
		return 2;
	}
	UINT threadf3(LPVOID PARAM){
		int* wait=(int*)PARAM;
		if (*wait<0)
			*wait =-(*wait);
		*wait=*wait>1000?1000:*wait;
		while(*wait>=0){
			Sleep(10);
			(*wait)--;
			cout<<"Still to wait for : "<<*wait<<endl;
		}
		cout<<"Thread3 will exit now!"<<endl;
		return 3;
	}

};

int main(){
	
	HANDLE hthread1;
	HANDLE hthread2;
	HANDLE hthread3;

	hthread1=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)threadf1,NULL,0,NULL);
	hthread2=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)threadf2,NULL,0,NULL);
	int N=100;
	hthread3=CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)threadf3,&N,0,NULL);
	Sleep(100);
	cout<<"Wait for thread3 to terminate!"<<endl;
	WaitForSingleObject(hthread3,-1);
	cout<<"farther is exiting!"<<endl;
	return 0;
}