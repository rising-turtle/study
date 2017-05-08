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
	void exitThread();
	void back(int );
	void done();
public slots:
	void consume(int );
	void finish();
};


#endif