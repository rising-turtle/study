#ifndef GUITHREAD_H
#define GUITHREAD_H

// #include <QThread>
#include <QWidget>
#include <QObject>
#include <QPushButton>
#include "Thread.h"

class CUIThread : public QWidget{
	Q_OBJECT
public:
	CUIThread(QWidget* parent = NULL);
	~CUIThread();
public slots:
	void startOrStopThreadA();
	void startOrStopThreadB();
public:
	QPushButton* m_btn_threadA;
	QPushButton* m_btn_threadB;
	QPushButton* m_btn_quit;
	
	Thread* m_thread_A;
	Thread* m_thread_B;
};



#endif
