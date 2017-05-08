#ifndef LOCKER_H
#define LOCKER_H

#include <QObject>
#include <QMutex>

class CLocker : public QObject
{
	Q_OBJECT
public:
	CLocker();
Q_SIGNALS:
	void jump(int);
public Q_SLOTS:
	void start();
	void change();
public:
	QMutex mutex;
	volatile int k;
};

#endif
