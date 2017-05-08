#include "GUIThread.h"
#include <QApplication>

int main(int argc, char* argv[]){
	QApplication app(argc,argv);

	CUIThread ui;
	ui.show();
	QObject::connect(ui.m_btn_quit,SIGNAL(clicked()),&app,SLOT(quit()));

	return app.exec();
}
