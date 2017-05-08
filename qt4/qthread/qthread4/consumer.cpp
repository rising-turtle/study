#include "consumer.h"
#include <iostream>
#include <QThread>
#include <fstream>
using namespace std;

CConsumer::CConsumer(){}

void CConsumer::consume(int n)
{
	static int cnt=0;
	static bool once = false;
	if(!once){
		cout<<"consumer thread: "<<(int)QThread::currentThreadId()<<" active!"<<endl;
		once = true;
	}
	// consumed();
	m_ticket = n;

	if(++cnt>5){
		cnt = 0;
		back(n);
	}
	/*if(n>100){
		cout<<"ticket is enough!"<<endl;
		done();
	}*/
}

void CConsumer::stopCon(){
	cout<<"thread consumer: "<<(int)QThread::currentThreadId()<<" exit!"<<endl;
	emit finished();
}

void CConsumer::record()
{
	cout<<"record in thread: "<<(int)QThread::currentThreadId()<<endl;
	ofstream record("debug.log");
	record<<"last ticket is: "<<m_ticket<<endl;
}
