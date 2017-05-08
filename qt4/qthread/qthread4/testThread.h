#ifndef CTHREAD_H
#define CTHREAD_H

#include <QObject>

class CThread : public QObject
{
	Q_OBJECT
public:
	CThread();
Q_SIGNALS:
	void back(int );
	void finished();
public slots:
	void consume(int );
	void finish();
};


#endif
