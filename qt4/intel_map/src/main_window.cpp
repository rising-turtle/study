#include "main_window.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QScrollArea>
#include "imagemap.h"
#include "graphics.h"

CMainWindow::CMainWindow(QWidget* parent){
	
	setWindowTitle("Show Intel Map");
	// initailize inner widgets
	m_btn_start = new QPushButton("start");
	m_btn_finalmap = new QPushButton("final");
	m_btn_quit = new QPushButton("quit");
	m_btn_dump = new QPushButton("dump");
	m_points_dis = new CPoints;
	m_final_map = new CMap2D;
	m_map_painter = new MapPainter;	

	resizeWindowSize(600,400);

// manage these widgets	
	QHBoxLayout* right_up = new QHBoxLayout;
	right_up->addWidget(m_btn_start);
	right_up->addWidget(m_btn_finalmap);
	right_up->addWidget(m_btn_quit);
	right_up->addWidget(m_btn_dump);

	QVBoxLayout * right = new QVBoxLayout;
	// right->addWidget(m_btn_start);
	// right->addWidget(m_btn_finalmap);
	// right->addStretch();
	// right->addWidget(m_final_map);
	right->addWidget(m_map_painter);
	right->addLayout(right_up);
	
	// QVBoxLayout* left = new QVBoxLayout;
	// left->addWidget(m_points_dis);
	// left->addWidget(left_btn);

	QHBoxLayout * hori = new QHBoxLayout;
	hori->addWidget(m_points_dis);
	// hori->addLayout(left);
	hori->addLayout(right);

	setLayout(hori);
	
	// set inner connections
	inner_connections();
	
}
CMainWindow::~CMainWindow(){}

void CMainWindow::inner_connections(){
	QObject::connect(m_final_map,SIGNAL(sendMap(CMap2D*)),m_map_painter,SLOT(receMap(CMap2D*)));
}

void CMainWindow::resizeWindowSize(int width, int height){
	
	if(width< 0 || height < 0)
	{
		return ;
	}
	
	resize(width,height);
	// m_points_dis->resize((int)(width*0.5),int(height*0.85));
	m_points_dis->setMinimumSize((int)(width*0.5),int(height*0.85));
	m_map_painter->resize((int)(width*0.45), int(height*0.5));
}


