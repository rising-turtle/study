#ifndef _MYWINDOW_H
#define _MYWINDOW_H

// #include "ui_second.h"

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>


#include "gen_points.h"
#include "points.h"
#include <QPushButton>

class MyWindow : public QWidget
{
	Q_OBJECT
	public:
		MyWindow(QWidget*parent=NULL);
		~MyWindow();
	public:
		void init_connections();
		void setWindowSize(int width, int height);
	public:
		CPoints * m_points_dis;
		QPushButton* m_btn_start; 
};


#endif


