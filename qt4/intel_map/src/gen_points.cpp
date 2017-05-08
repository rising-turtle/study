#include "gen_points.h"
#include <stdlib.h>
#include <QTimer>
#include <iostream>
#include <ctime>
#include <cmath>
#include <algorithm>
// #include <QMessageBox.h>

const float bad_range = -100000; // to indicate bad range value
namespace{
	bool isBadRange(float r){
		return (r==bad_range);
	}
}


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


CGenPoints::CGenPoints():m_px(N),m_py(N),m_pz(N),mbStop(false),mfin(NULL){
	// set a timer to continueously genertePoints
	m_updateTimer = new QTimer(this);
	// QObject::connect(m_updateTimer, SIGNAL(timeout()),this,SLOT(generatePoints()));
}

void CGenPoints::genEllipsold(){
	float a = fabs(genrand());
	float b = fabs(genrand());
	float x = fabs(genrand());
	float y = fabs(genrand());
	float theta = fabs(genrand());
	cout<<"genEllipsold (x,y,t,a,b): "<<"("<<x<<","<<y<<","<<theta<<","<<a<<","<<b<<")"<<endl;
	sendEllipsold(x,y,theta,a,b);
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
	if(!mfin || !mfin->is_open()){
		string datafile = "/home/lyxp/work/qt4/intel_map/data/intel-lab.log";//"../../data/intel-lab.log";
		mfin = new ifstream(datafile.c_str());
		if(!mfin->is_open()){
		  cout<<"this stream is not right!"<<endl;
		}
		else
		  cout<<"succeed stream!"<<endl;
	}
	else{
		cout<<"file has already been opened!"<<endl;
		return ;
	}
	// begin to read data from file
	synread();
}

void CGenPoints::next(){
	static bool bInit = false;
	static ifstream* fin;/* = new ifstream("/home/lyxp/work/qt4/intel_map/data/intel-lab.log");*/
	if(!bInit){
		bInit = true;
		string datafile = "/home/lyxp/work/qt4/intel_map/data/intel-lab.log";//"../../data/intel-lab.log";
		fin = new ifstream(datafile.c_str());
		if(!fin->is_open()){
		  cout<<"this stream is not right!"<<endl;
		}
		else
		  cout<<"succeed stream!"<<endl;
	}
	if(!readOneLineIntel(fin,"ROBOT")){
		// QMessagebox::information(NULL,"Title","Finished!");
	}
}

void CGenPoints::stop(){
	mbStop = !mbStop;
	cout<<"##############stop: "<<mbStop<<"##############"<<endl;
	synread();
}

void CGenPoints::synread(){
	if(!mbStop){
	   if(!readOneLineIntel(mfin,"ROBOT")) 
	 	{
			cout<<"end of the stream!"<<endl;
			mbStop = true;
			return ;
		}
		// Send this frame to display
		sendMapInfo(&m_px[0],&m_py[0],m_px.size(),&m_rx,&m_ry,&m_th);
	}
}

bool CGenPoints::readOneLineIntel(ifstream *fin, string tag)
{
	if(!fin->is_open()){
		cout<<"file stream is not right!"<<endl;
		return false;
	}
	bool ret = false;
	char line[8192];
	double timestamp;
	double start, fov, resolution, maxRange, accuracy;
	int laserType, remissionMode, num_points;
	float rx,ry,rth; // robot pose
	while(fin->getline(line,8192)){
		string tmptag(strtok(line," "));
		if(!equal(tag.begin(),tag.end(),tmptag.begin()))
		{ 
			continue; // not the line we wanted	
		}
		laserType = (int)(atof(strtok(NULL," ")));	
		start = atof(strtok(NULL," "));
		fov = atof(strtok(NULL," "));
		resolution = atof(strtok(NULL," "));
		maxRange = atof(strtok(NULL," "));
		accuracy = atof(strtok(NULL," "));
		remissionMode = atoi(strtok(NULL," "));
		num_points = atoi(strtok(NULL," "));
		
		vector<float> bearing(num_points); // BEARING DATA
		for(int i=0;i<num_points;i++){
			bearing[i]=atof(strtok(NULL," "));
			if(bearing[i]>maxRange){
				bearing[i] = 0; // bad bearing value 
			}
		}

		// remission value
		atof(strtok(NULL," "));

		// ROBOT POSE
		rx = atof(strtok(NULL," "));
		ry = atof(strtok(NULL," "));
		rth = atof(strtok(NULL," "));

		// Translate to global framework
		translate2GlobalFrame(bearing,rx,ry,rth);
		
		// save these info to send
		m_bearing.swap(bearing);
		m_rx = rx;
		m_ry = ry;
		m_th = rth;
		
		// save for the final map filtering the bad points
		m_gx.push_back(m_px);
		m_gy.push_back(m_py);
		m_trx.push_back(m_rx);
		m_try.push_back(m_ry);
		m_tth.push_back(m_th);

		ret = true;
		break;
	}
	return ret;
}

void CGenPoints::translate2GlobalFrame(vector<float>& bearing, float rx, float ry, float th)
{
#define BEAR_NUM 180
#define PI 3.141592654
	static bool initAngle = false;
	static float cosAngle[BEAR_NUM];
	static float sinAngle[BEAR_NUM];
	static float Angle[BEAR_NUM];
	if(!initAngle){
		initAngle = true;
		static float minAngle =  -(PI)/2.0;  // 0.0;
		static float dfi = (float)((PI)/180.0);
		for(int i=0;i<BEAR_NUM;i++){
			Angle[i]=minAngle + dfi*i;
			cosAngle[i] = cosf(Angle[i]);
			sinAngle[i] = sinf(Angle[i]);
		}
	}
	if(bearing.size()!=BEAR_NUM){
		cout<<"something is wrong!"<<endl;
	}
	if(m_px.size()!=BEAR_NUM){
		m_px.resize(BEAR_NUM);
		m_py.resize(BEAR_NUM);
	}

	float x,y,fx,fy,nIdx,nIdy;
	float fcos = cosf(th);
	float fsin = sinf(th);
	for(int i=0;i<BEAR_NUM;i++){
		if(bearing[i]==0.0){ // this is bad range 
			m_px[i]=m_py[i]=bad_range;
			continue;
		}
		x=bearing[i]*cosAngle[i];
		y=bearing[i]*sinAngle[i];
		fx=fcos*x-fsin*y+rx;
		fy=fsin*x+fcos*y+ry;

		m_px[i] = fx;
		m_py[i] = fy;
	}
}


void CGenPoints::drawFinalMap()
{
	// send all the map info frame by frame
	vector<float>::iterator it;
	for(int i=0;i<m_gx.size();i++)
	{
		// filter bad range
		it = remove_if(m_gx[i].begin(),m_gx[i].end(),(bool(*)(float))isBadRange);
		if(it!=m_gx[i].end()){
			m_gx[i].erase(it,m_gx[i].end());
			it = remove_if(m_gy[i].begin(),m_gy[i].end(),(bool(*)(float))isBadRange);
			m_gy[i].erase(it,m_gy[i].end());
		}
		// send this frame to update Final Map
		sendGlbalMapInfo(&m_gx[i][0],&m_gy[i][0],m_gx[i].size(),m_trx[i],m_try[i],m_tth[i]);		
	}

	// send n=-1 to end sending map info
	sendGlbalMapInfo(NULL,NULL,-1,0,0,0);
}
