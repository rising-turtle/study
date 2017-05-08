#ifndef POINTS_H
#define POINTS_H

#include <QObject>
#include <QGLWidget>
#include <QWidget>
#include <QWheelEvent>
#include <QMouseEvent>
#include <vector>

using namespace std;

class CPoints : public QGLWidget
{
	Q_OBJECT
public:
	CPoints(QWidget* parent=0);
protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();
public Q_SLOTS:
	void receMapInfo(float* fx, float* fy, int n, float* rx, float* ry, float* th);
	void recePoints(float* px, float* py, float* pz, int n);
	void receEllipsold(float,float,float,float,float);
Q_SIGNALS:
	void synRead();
protected:
	virtual void wheelEvent(QWheelEvent* e); 
	virtual void mousePressEvent(QMouseEvent* e);
	virtual void mouseReleaseEvent(QMouseEvent* e);
private:
	void drawPoints();
	void drawEllipsold();
	void drawMap();
	void draw();
	float computeY(float,float,float);
	void computeTransformation(float,float,float,vector<float>&,vector<float>&);
private:
	// map points
	std::vector<float> m_px;
	std::vector<float> m_py;
	std::vector<float> m_pz;
	
	// robot pose 
	std::vector<float> m_rx;
	std::vector<float> m_ry;
	std::vector<float> m_th;
	
	// ellipsold info
	vector<vector<float> > m_ex;
	vector<vector<float> > m_ey;
	vector<float> m_ori_x;
	vector<float> m_ori_y;
	
	// scale parameters
	float m_xScale;
	float m_yScale;
	
	// mouse drag event
	int m_last_x;
	int m_last_y;
	int m_curr_x;
	int m_curr_y;
	float trans_x;
	float trans_y;

	bool m_IsReady;
};

#endif
