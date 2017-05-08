#include "ui_win.h"
#include "producer.h"
#include "consumer.h"
#include "testThread.h"
#include <QHBoxLayout>
#include <iostream>

using namespace std ;

CUIWin::CUIWin(){
	resize(100,100);
	m_btn_start = new QPushButton("start");
	m_btn_stop = new QPushButton("stop");
	m_btn_test = new QPushButton("test");
	QHBoxLayout* hlay = new QHBoxLayout;
	hlay->addWidget(m_btn_start);
	hlay->addWidget(m_btn_stop);
	hlay->addWidget(m_btn_test);
	setLayout(hlay);

	setSystem();

}

CUIWin::~CUIWin(){
	// delete m_pro;
	// delete m_con;
}

void CUIWin::startSystem(){
	m_thread_pro->start();
	m_thread_con->start();
	m_thread_test->start();
	cout<<"System is on!"<<endl;
}

void CUIWin::stopTest(){
	if(m_thread_test->isRunning()){
		cout<<"thread_test is running!"<<endl;
	}
	// m_thread_test->quit();
	stopThreadTest();
	m_thread_test->wait(5000);
	if(m_thread_test->isRunning()){
		cout<<"thread_test is still running!"<<endl;
		m_thread_test->quit();
		cout<<"wait for thread_test quit!"<<endl;
		m_thread_test->wait();
	}
	if(!m_thread_test->isRunning()){
		cout<<"succeed to quit thread_test!"<<endl;
	}else{
		cout<<"This is ca!!!!!!"<<endl;
	}
}

void CUIWin::stopAll(){
	cout<<"!!!!stop app now!!"<<endl;
	stopApp();
}

void CUIWin::stopSystem(){
	stopThreadPro();
	// m_thread_pro->quit();
	m_thread_pro->wait(5000);

	if(m_thread_pro->isRunning())
	{	
		cout<<"thread_pro is still running, terminate!"<<endl;
		m_thread_pro->quit();
		// cout<<"wait for thread_pro to terminate!"<<endl;
		m_thread_pro->wait();
	}else{
		cout<<"thread_pro is succeed to stop!"<<endl;
	}
	cout<<"$$$$$ stop thread pro! $$$$$"<<endl;
	/*
	stopThreadCon();
	cout<<"$$$$$ stop thread con! $$$$$"<<endl;
	// m_thread_con->quit();
	m_thread_con->wait(5000);
	if(m_thread_con->isRunning())
	{	
		cout<<"thread_con is still running, terminate!"<<endl;
		m_thread_con->quit();
		// cout<<"wait for thread_con to terminate!"<<endl;
		m_thread_con->wait();
	}else{
		cout<<"thread_con is succeed to stop!"<<endl;
	}
	stopApp();*/
}

void CUIWin::setSystem()
{
	m_thread_pro = new QThread;
	m_thread_con = new QThread;
	m_thread_test = new QThread;
	m_pro = new CProducer; // while(1)
	m_con = new CConsumer; // eventLoop
	m_test = new CThread;

	m_pro->moveToThread(m_thread_pro);
	m_con->moveToThread(m_thread_con);
	m_test->moveToThread(m_thread_test);

	// this may not work for thread pro
	QObject::connect(m_thread_pro,SIGNAL(started()),m_pro,SLOT(produce()));
	QObject::connect(m_pro,SIGNAL(finished()),m_thread_pro,SLOT(quit()),Qt::DirectConnection);
//	QObject::connect(m_pro,SIGNAL(finished()),m_pro,SLOT(deleteLater()));
//	QObject::connect(m_thread_pro,SIGNAL(finished()),m_thread_pro,SLOT(deleteLater()));
	
	// for thread con
	QObject::connect(m_con,SIGNAL(finished()),m_thread_con,SLOT(quit()),Qt::DirectConnection);
//	QObject::connect(m_con,SIGNAL(finished()),m_con,SLOT(deleteLater()));
//	QObject::connect(m_thread_con,SIGNAL(finished()),m_thread_con,SLOT(deleteLater()));
	
	// for test
	QObject::connect(m_test,SIGNAL(finished()),m_thread_test,SLOT(quit()),Qt::DirectConnection);
	// for communicate
	QObject::connect(m_pro,SIGNAL(produced(int)),m_con,SLOT(consume(int)));
	QObject::connect(m_con,SIGNAL(back(int)),m_pro,SLOT(back(int)),Qt::DirectConnection);

	// for control
	QObject::connect(m_btn_start,SIGNAL(clicked()),this,SLOT(startSystem()));
	QObject::connect(m_btn_stop,SIGNAL(clicked()),this,SLOT(stopSystem()));

	QObject::connect(m_btn_test,SIGNAL(clicked()),this,SLOT(stopTest()));
	

	QObject::connect(this,SIGNAL(stopThreadPro()),m_pro,SLOT(done()),Qt::DirectConnection);
	QObject::connect(this,SIGNAL(stopThreadCon()),m_con,SLOT(stopCon()));
	QObject::connect(this,SIGNAL(stopThreadTest()),m_test,SLOT(finish()));

	// when quit thread1 -> thread2 -> record & thread3
	QObject::connect(m_thread_pro,SIGNAL(finished()),m_con,SLOT(record()));
	QObject::connect(m_thread_pro,SIGNAL(finished()),m_con,SLOT(stopCon()));
	QObject::connect(m_thread_con,SIGNAL(finished()),m_test,SLOT(finish()));
	QObject::connect(m_thread_test,SIGNAL(finished()),this,SLOT(stopAll()));
}
