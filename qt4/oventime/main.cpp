#include <QApplication>
#include "oventime.h"

int main(int argc, char* argv[])
{
	QApplication app(argc,argv);
	OvenTime* pOven = new OvenTime;
	pOven->show();
	return app.exec();

}
