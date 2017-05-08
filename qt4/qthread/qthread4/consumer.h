#ifndef CONSUMER_H
#define CONSUMER_H

#include <QObject>

class CConsumer : public QObject
{
	Q_OBJECT
public:
	CConsumer();
Q_SIGNALS:
	void consumed();
	void back(int );
	void finished();
public slots:
	void consume(int );
	void stopCon();
	void record();
public:
	volatile int m_ticket;
};


#endif
