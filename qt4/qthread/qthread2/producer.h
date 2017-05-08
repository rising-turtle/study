#ifndef PRODUCER_H
#define PRODUCER_H

#include <QObject>

class CProducer : public QObject
{
	Q_OBJECT
public:
	CProducer();
Q_SIGNALS:
	void produced(int );
	void finish();
public slots:
	void produce();
private:
	volatile int m_ticket;
	int m_all_ticket;
};

#endif
