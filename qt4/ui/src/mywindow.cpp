#include "mywindow.h"

// three kinds of style for managing widgets
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>


MyWindow::MyWindow(QWidget* parent){
	
	setWindowTitle("Random Points");
	setWindowSize(200,160);

	// construct all its members
	m_btn_start = new QPushButton("start");
	m_points_dis = new CPoints;
	
	// manage layout of these widgets
	QVBoxLayout *vertical = new QVBoxLayout;
	vertical->addWidget(m_points_dis);
	vertical->addWidget(m_btn_start);
	setLayout(vertical);

	// init connections
	init_connections();
}

MyWindow::~MyWindow(){}

// connects signals between inner widgets 
void MyWindow::init_connections()
{
}

void MyWindow::setWindowSize(int width, int height){
	if(width <0 || height <0){
		return ;
	}
	resize(width,height);
	m_points_dis->resize((int)(width*0.75),(int)(height*0.6));
}


