#include "testThread.h"
#include <iostream>
#include <QThread>
using namespace std;

CThread::CThread(){}

void CThread::consume(int n)
{
	static int cnt=0;
	static bool once = false;
	if(!once){
		cout<<"consumer thread: "<<(int)QThread::currentThreadId()<<" active!"<<endl;
		once = true;
	}
}

void CThread::finish(){
	cout<<"finish threadTest: "<<(int)QThread::currentThreadId()<<endl;
	finished();
}

