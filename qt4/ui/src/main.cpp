#include <QApplication>
#include <QObject>
#include <QGLFormat>
#include <QPushButton>
#include <iostream>
#include "points.h"
#include "gen_points.h"
#include "mywindow.h"

int main(int argc, char* argv[])
{
	QApplication app(argc,argv);
	if(!QGLFormat::hasOpenGL()){
		std::cerr<<"No OpenGL!"<<std::endl;
		return -1;
	}
	/*CPoints tmp;
	tmp.setWindowTitle("Points");
	tmp.resize(200,300);
	*/
	MyWindow ui_window;
	CGenPoints generate;
	QObject::connect(&generate,SIGNAL(sendpoints(float*,float*,float*,int)),ui_window.m_points_dis,SLOT(recePoints(float*,float*,float*,int)));
	
	/*QPushButton * button = new QPushButton("Start");*/
	QObject::connect(ui_window.m_btn_start, SIGNAL(clicked()),&generate,SLOT(start()));
		
	// button->show();
	// tmp.show();
	ui_window.show();
	
	return app.exec();
}
