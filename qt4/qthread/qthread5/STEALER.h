#ifndef STEALER_H
#define STEALER_H
#include <QObject>


class CStealer: public QObject
{
	Q_OBJECT
public:
	CStealer();
public Q_SLOTS:
	void trap(int);
	void start();
Q_SIGNALS:
	void steal();
public:
	volatile bool succeed;
};


#endif
