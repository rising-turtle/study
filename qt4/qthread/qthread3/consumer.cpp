#include "consumer.h"
#include <iostream>
#include <QThread>
using namespace std;

CConsumer::CConsumer(){}

void CConsumer::consume(int n)
{
	static int cnt=0;
	cout<<"thread: "<<(int)QThread::currentThreadId()<<" get ticket: "<<n<<endl;
	// consumed();

	if(++cnt>5){
		cnt = 0;
		back(n);
	}
	if(n>100){
		cout<<"ticket is enough!"<<endl;
		done();
	}
}

void CConsumer::finish()
{
	cout<<"No ticket is avaliable!"<<endl;
	cout<<"thread: "<<(int)QThread::currentThreadId()<<" exit!"<<endl;
	exitThread();
}

