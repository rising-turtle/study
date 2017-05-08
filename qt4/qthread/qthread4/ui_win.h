#ifndef UI_WIN_H
#define UI_WIN_H

#include <QWidget>
#include <QPushButton>
#include <QThread>

class CProducer;
class CConsumer;
class CThread;

class CUIWin : public QWidget
{
	Q_OBJECT
public:
	CUIWin();
	~CUIWin();
	void setSystem();
public Q_SLOTS:
	void startSystem();
	void stopSystem();
	void stopTest();
	void stopAll();
Q_SIGNALS:
	void stopThreadPro();
	void stopThreadCon();
	void stopApp();
	void stopThreadTest();
public:
	QPushButton* m_btn_start;
	QPushButton* m_btn_stop;
	QPushButton* m_btn_test;
	QThread* m_thread_pro;
	QThread* m_thread_con;
	QThread* m_thread_test;
	CProducer* m_pro;
	CConsumer* m_con;
	CThread* m_test;
};

#endif
