#include <QApplication>
#include <QObject>

#include "ui_win.h"
#include <iostream>

int main(int argc,char* argv[]){
	QApplication app(argc,argv);
	CUIWin ui;
	ui.show();
	QObject::connect(&ui,SIGNAL(stopApp()),&app,SLOT(quit()));

	return app.exec();
}
