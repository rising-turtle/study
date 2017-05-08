#include "producer.h"
#include <iostream>
#include <QMutexLocker>
#include <QThread>

using namespace std;

CProducer::CProducer():m_ticket(0),m_all_ticket(100),m_done(false){}

void CProducer::produce()
{
	while(1){
		QMutexLocker locker(&m_mutex);
		if(m_done) break;
		cout<<"thread: "<<(int)QThread::currentThreadId()<<" selling ticket: "<<++m_ticket<<endl;
		produced(m_ticket);
	}
	cout<<"producer exit!"<<endl;
	finish();
}
void CProducer::back(int n){
	QMutexLocker locker(&m_mutex);
	cout<<"thread: "<<(int)QThread::currentThreadId()<<" return ticjet: "<<n<<endl;
}
void CProducer::done(){
	QMutexLocker locker(&m_mutex);
	cout<<"thread: "<<(int)QThread::currentThreadId()<<" !!! all tickets done #####"<<endl;
	m_done = true;
}
