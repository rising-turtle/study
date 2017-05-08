#ifndef POINTS_H
#define POINTS_H

#include <QObject>
#include <QGLWidget>
#include <QWidget>
#include <vector>

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
	void recePoints(float* px, float* py, float* pz, int n);
private:
	void draw();
private:
	std::vector<float> m_px;
	std::vector<float> m_py;
	std::vector<float> m_pz;
	bool m_IsReady;
};

#endif
