#include <QApplication>
#include <QObject>
#include <QGLFormat>
#include <QPushButton>
#include <iostream>
#include "points.h"
#include "gen_points.h"

int main(int argc, char* argv[])
{
	QApplication app(argc,argv);
	if(!QGLFormat::hasOpenGL()){
		std::cerr<<"No OpenGL!"<<std::endl;
		return -1;
	}
	CPoints tmp;
	tmp.setWindowTitle("Points");
	tmp.resize(200,300);

	CGenPoints generate;
	QObject::connect(&generate,SIGNAL(sendpoints(float*,float*,float*,int)),&tmp,SLOT(recePoints(float*,float*,float*,int)));
	
	QPushButton * button = new QPushButton("Start");
	QObject::connect(button, SIGNAL(clicked()),&generate,SLOT(start()));
	button->show();
	tmp.show();
	return app.exec();
}
