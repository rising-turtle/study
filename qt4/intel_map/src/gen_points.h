#ifndef GEN_POINTS_H
#define GEN_POINTS_H

#include <algorithm>
#include <numeric>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <QWidget>
#include <QTimer>
#include <QObject>

using namespace std;

class CGenPoints: public QWidget
{
	Q_OBJECT
public:
	CGenPoints();
public Q_SLOTS:
	void generatePoints();
	void start();
	void next();
	void stop();
	void synread();
	void drawFinalMap();
	void genEllipsold();
	
Q_SIGNALS:
	void sendpoints(float* px, float *py, float* pz, int n);
	void sendEllipsold(float, float, float,float,float);
	void sendMapInfo(float* px, float *py, int n, float* rx, float* ry, float* rth); // send map info
	void sendGlbalMapInfo(float* gx, float* gy, int n, float rx, float ry, float rth ); // send global map info
public:
	bool readOneLineIntel(ifstream* fin, string tag="ROBOT");
	void translate2GlobalFrame(vector<float>& bearing, float rx, float ry, float th);
public:
	vector<float> m_bearing;
	float m_rx;
	float m_ry;
	float m_th;
private:
	std::vector<float> m_px;
	std::vector<float> m_py;
	std::vector<float> m_pz;
	QTimer * m_updateTimer;
	bool mbStop;
	ifstream* mfin; // input file
private:
	// for the final MAP
	vector<vector<float> > m_gx;
	vector<vector<float> > m_gy;
	vector<float> m_trx;
	vector<float> m_try;
	vector<float> m_tth;
};


#endif
