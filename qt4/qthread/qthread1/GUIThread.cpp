#include "GUIThread.h"
#include <QHBoxLayout>

CUIThread::CUIThread(QWidget* parent)
{
	setWindowTitle("Thread1");
	m_btn_threadA = new QPushButton("Start A");
	m_btn_threadB = new QPushButton("Start B");
	m_btn_quit = new QPushButton("Quit");
	
	m_thread_A = new Thread;
	m_thread_A->setMessage("A");
	m_thread_B = new Thread;
	m_thread_B->setMessage("B");

	QObject::connect(m_btn_threadA,SIGNAL(clicked()),this,SLOT(startOrStopThreadA()));
	QObject::connect(m_btn_threadB,SIGNAL(clicked()),this,SLOT(startOrStopThreadB()));

	resize(200,100);
	// resizeWindowSize(200,100);

	// 
	QHBoxLayout * hori1 = new QHBoxLayout;
	hori1->addWidget(m_btn_threadA);
	hori1->addWidget(m_btn_threadB);
	hori1->addWidget(m_btn_quit);
	
	setLayout(hori1);
}

CUIThread::~CUIThread(){}

void CUIThread::startOrStopThreadA(){
	if(m_thread_A->isRunning()){
		m_thread_A->stop();
		m_btn_threadA->setText(tr("Start A"));
	}
	else{
		m_thread_A->start();
		m_btn_threadA->setText(tr("Stop A"));
	}
}

void CUIThread::startOrStopThreadB(){
	if(m_thread_B->isRunning()){
		m_thread_B->stop();
		m_btn_threadB->setText(tr("Start B"));
	}
	else{
		m_thread_B->start();
		m_btn_threadB->setText(tr("Stop B"));
	}
}
