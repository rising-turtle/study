#include "points.h"
#include <iostream>

CPoints::CPoints(QWidget* parent):QGLWidget(parent),m_IsReady(false)
{
	setFormat(QGLFormat(QGL::DoubleBuffer | QGL::DepthBuffer));
}

void CPoints::initializeGL()
{
	qglClearColor(Qt::black);
	glShadeModel(GL_FLAT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
}

void CPoints::resizeGL(int width, int height)
{
	glViewport(0,0,width,height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	GLfloat x = GLfloat(width)/ height;
	glFrustum(-x,+x,-1.0,1.0,4.0,15.0);
	glMatrixMode(GL_MODELVIEW);
}

void CPoints::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	draw();
}


void CPoints::recePoints(float* px, float* py, float* pz, int n)
{	
	m_IsReady = false;
	if(m_px.size()<n)
	{
		m_px.resize(n);
		m_py.resize(n);
		m_pz.resize(n);
	}
	copy(px,px+n,m_px.begin());
	copy(py,py+n,m_py.begin());
	copy(pz,pz+n,m_pz.begin());
	m_IsReady = true;
	std::cout<<"receive points!"<<std::endl;
	updateGL();
}

void CPoints::draw(){
	if(!m_IsReady)
		return;
	// std::cout<<"drawing!"<<std::endl;
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0,0,-5.0);
	
	glBegin(GL_POINTS);
	qglColor(Qt::red);
	for(int i=0;i<m_px.size();i++)
		glVertex3f(m_px[i],m_py[i],/*m_pz[i]*/0);
	glEnd();
}






/*
void CPoints::draw(){
	static const GLfloat P1[3] = {0.0,-1.0,+2.0};
	static const GLfloat P2[3] = {+1.7,-1.0,-1.0};
	static const GLfloat P3[3] = {-1.7,-1.0,-1.0};
	static const GLfloat* const coords[1][3] ={{P1,P2,P3}};
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0,0,-10.0);
	
	glBegin(GL_TRIANGLES);
	qglColor(Qt::red);
	for(int i=0;i<3;i++)
		glVertex3f(coords[0][i][0],coords[0][i][1],coords[0][i][2]);
	glEnd();
}*/
