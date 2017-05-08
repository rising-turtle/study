#include <QCoreApplication>
#include "STEALER.h"
#include "LOCKER.h"
#include <QThread>

int main(int argc, char* argv[])
{
	QCoreApplication app(argc,argv);
	CLocker mylock;
	CStealer stealer;
	QObject::connect(&mylock,SIGNAL(jump(int)),&stealer,SLOT(trap(int)),Qt::DirectConnection);
	QObject::connect(&stealer,SIGNAL(steal()),&mylock,SLOT(change()),Qt::DirectConnection);
	
	QThread thread_lock;
	QThread thread_steal;
	mylock.moveToThread(&thread_lock);
	stealer.moveToThread(&thread_steal);
	
	QObject::connect(&thread_lock,SIGNAL(started()),&mylock,SLOT(start()));
	QObject::connect(&thread_steal,SIGNAL(started()),&stealer,SLOT(start()));
	
	thread_lock.start();	
	thread_steal.start();

	return app.exec();
}



