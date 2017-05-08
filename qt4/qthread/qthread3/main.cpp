#include <QObject>
#include <QApplication>
#include <QThread>
#include "consumer.h"
#include "producer.h"


int main(int argc, char* argv[])
{
	QApplication app(argc,argv);
	
	CProducer pro;
	CConsumer con;
	QObject::connect(&pro,SIGNAL(produced(int)),&con,SLOT(consume(int)));
	QObject::connect(&con,SIGNAL(consumed()),&pro,SLOT(produce()));
	
	QObject::connect(&pro,SIGNAL(finish()),&con,SLOT(finish()));
	QObject::connect(&con,SIGNAL(back(int)),&pro,SLOT(back(int)));
	QObject::connect(&con,SIGNAL(done()),&pro,SLOT(done()),Qt::DirectConnection);

	// both their own threads
	QThread proThread;
	QThread conThread;

	pro.moveToThread(&proThread);
	con.moveToThread(&conThread);

	// con-> conThread -> proThread -> app
	QObject::connect(&proThread,SIGNAL(started()),&pro,SLOT(produce()));
	QObject::connect(&con,SIGNAL(exitThread()),&conThread,SLOT(quit()));
	QObject::connect(&conThread,SIGNAL(finished()),&proThread,SLOT(quit()));
	QObject::connect(&proThread,SIGNAL(finished()),&app,SLOT(quit()));
	
	proThread.start();
	conThread.start();

	return app.exec();
}
