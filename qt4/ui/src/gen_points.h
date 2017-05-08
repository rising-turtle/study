#ifndef GEN_POINTS_H
#define GEN_POINTS_H

#include <algorithm>
#include <numeric>
#include <vector>
#include <QWidget>
#include <QTimer>
#include <QObject>

class CGenPoints: public QWidget
{
	Q_OBJECT
public:
	CGenPoints();
public Q_SLOTS:
	void generatePoints();
	void start();
Q_SIGNALS:
	void sendpoints(float* px, float *py, float* pz, int n);
private:
	std::vector<float> m_px;
	std::vector<float> m_py;
	std::vector<float> m_pz;
	QTimer * m_updateTimer;
};


#endif
