#ifndef PRODUCER_H
#define PRODUCER_H

#include <QObject>
#include <QMutex>

class CProducer : public QObject
{
	Q_OBJECT
public:
	CProducer();
Q_SIGNALS:
	void produced(int );
	void finished();
public slots:
	void produce();
	void back(int );
	void done();
private:
	volatile int m_ticket;
	int m_all_ticket;
	volatile bool m_done;
	QMutex m_mutex;
};

#endif
