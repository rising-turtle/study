#include "mainwin.h"

#include <QObject>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>

MyWin::~MyWin(){}
MyWin::MyWin()
{
	m_left = new QPushButton("YD");
	// m_left = new QWidget;
	m_cancle = new QPushButton("cancle");
	m_sa = new QScrollArea();
	m_pw = new PixmapWidget(QString("test.png"),m_sa);
	m_sa->setWidgetResizable(1);
	m_sa->setWidget(m_pw);

	/*
	QVBoxLayout* right = new QVBoxLayout;
	right->addWidget(m_sa);
	// right->addStretch();
	right->addWidget(m_cancle);
	
	QHBoxLayout* hori = new QHBoxLayout;
	hori->addWidget(m_left);
	hori->addLayout(right);
*/
	QHBoxLayout* hori = new QHBoxLayout;
	hori->addWidget(m_left);
	hori->addWidget(m_cancle);
	
	QVBoxLayout* verti = new QVBoxLayout;
	verti->addWidget(m_sa);
	verti->addLayout(hori);

	setLayout(verti);
	resize(400,300);
	m_left->resize(width()*0.5,height()*0.5);
	m_sa->resize(width()*0.5, height()*0.7);

}
