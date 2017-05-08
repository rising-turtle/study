#include "gen_points.h"
#include <stdlib.h>
#include <QTimer>
#include <iostream>
#include <ctime>

const double f_min = -5.0;
const double f_max = +5.0;
const int N = 2000;

float genrand(){
	static bool bfisrt = true;
	if(bfisrt)
	{
		srand((unsigned int)time(NULL));
		bfisrt = false;
	}
	return ((float)(rand()%10000))/1000.0 - 5.0;
}


CGenPoints::CGenPoints():m_px(N),m_py(N),m_pz(N){
	// set a timer to continueously genertePoints
	m_updateTimer = new QTimer(this);
	QObject::connect(m_updateTimer, SIGNAL(timeout()),this,SLOT(generatePoints()));
}

void CGenPoints::generatePoints(){
	// randomly generte N points, and then send them to paint
	generate(m_px.begin(),m_px.end(),genrand);
	generate(m_py.begin(),m_py.end(),genrand);
	generate(m_pz.begin(),m_pz.end(),genrand);
	sendpoints(&m_px[0],&m_py[0],&m_pz[0],N);
	
	std::cout<<"points are:" <<std::endl;
	for(int i=0;i<10;i++)
	{
		std::cout<<m_px[i]<<" "<<m_py[i]<<" "<<m_pz[i]<<std::endl;
	}

	std::cout<<"send points!"<<std::endl;
	return ;
}

void CGenPoints::start(){
	m_updateTimer->start(2000);
}

