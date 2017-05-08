#include "producer.h"
#include <iostream>
#include <QMutexLocker>
#include <QThread>

using namespace std;

CProducer::CProducer():m_ticket(0),m_all_ticket(100),m_done(false){}

void CProducer::produce()
{
	cout<<"Thread producer: "<<(int)QThread::currentThreadId()<<" active!"<<endl;
	while(1){
		QMutexLocker locker(&m_mutex);
		if(m_done) break;
		produced(++m_ticket);
		// QThread::msleep(10);
		QThread::yieldCurrentThread();
	}
	cout<<"Thread producer: "<<(int)QThread::currentThreadId()<<" exit!"<<endl;
}
void CProducer::back(int n){
	QMutexLocker locker(&m_mutex);
	static bool once = false;
	if(!once){
		cout<<"thread: "<<(int)QThread::currentThreadId()<<" return ticjet: "<<n<<endl;
		once = true;
	}
}
void CProducer::done(){
	//QMutexLocker locker(&m_mutex);
	cout<<"thread: "<<(int)QThread::currentThreadId()<<" stop producer!"<<endl;
	m_done = true;
	cout<<"last ticket: "<<m_ticket<<endl;
	finished();
}
