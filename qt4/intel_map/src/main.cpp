#include <QApplication>
#include <QObject>
#include <QGLFormat>
#include <QPushButton>
#include <iostream>
#include "points.h"
#include "gen_points.h"
#include "map2d.h"
#include "main_window.h"

int main(int argc, char* argv[])
{
	QApplication app(argc,argv);
	if(!QGLFormat::hasOpenGL()){
		std::cerr<<"No OpenGL!"<<std::endl;
		return -1;
	}

	CMainWindow ui;
	CGenPoints generate;

	QObject::connect(ui.m_btn_start,SIGNAL(clicked()),&generate,SLOT(start()));
	QObject::connect(&generate,SIGNAL(sendMapInfo(float*, float*,int , float*,float*, float*)),ui.m_points_dis,SLOT(receMapInfo(float*, float*,int , float*,float*, float*)));
	QObject::connect(ui.m_points_dis,SIGNAL(synRead()),&generate,SLOT(synread()));
	QObject::connect(&generate,SIGNAL(sendGlbalMapInfo(float*, float*,int, float, float, float)),ui.m_final_map,SLOT(updateMap(float*, float*, int, float, float, float)));
/*
	QObject::connect(ui.m_btn_start,SIGNAL(clicked()),&generate,SLOT(genEllipsold()));
	QObject::connect(&generate,SIGNAL(sendEllipsold(float,float,float,float,float)),ui.m_points_dis,SLOT(receEllipsold(float,float,float,float,float)));*/
	QObject::connect(ui.m_btn_finalmap,SIGNAL(clicked()),&generate,SLOT(drawFinalMap()));
	QObject::connect(ui.m_btn_dump,SIGNAL(clicked()),ui.m_map_painter->imagewidget,SLOT(dumpscreen()));
	QObject::connect(ui.m_btn_quit,SIGNAL(clicked()),&app,SLOT(quit()));
	
	ui.show();
	return app.exec();
}
