#include "ZHIcp_Warpper.h"

#include <mrpt/slam.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::slam;

namespace {
	struct CICP::TReturnInfo* getRet(){
		static struct CICP::TReturnInfo* g_pRet = new CICP::TReturnInfo;
		return g_pRet;
	}
}

CICPWarpper::CICPWarpper():
m_pICP(new CICP),
m_pTrans(new CPosePDFGaussian),
m_pPose(new CPose2D),
skip_window(false), // true
ICP_method((int)icpClassic)
//m_pRet(new TReturnInfo)
{
	initICP();
}
CICPWarpper::~CICPWarpper(){
	delete m_pICP;
	// delete m_pRet;
	delete m_pTrans;
	delete m_pPose;
}

void CICPWarpper::initICP(){
	m_pICP->options.ICP_algorithm = (TICPAlgorithm)ICP_method;

	m_pICP->options.maxIterations			= 100;
	m_pICP->options.thresholdAng			= DEG2RAD(10.0f);
	m_pICP->options.thresholdDist			= 0.75f;
	m_pICP->options.ALFA					= 0.5f;
	m_pICP->options.smallestThresholdDist	= 0.05f;
	m_pICP->options.doRANSAC = false;
	//m_pICP->options.dumpToConsole();
}



float CICPWarpper::ICP(CSimplePointsMap& m1, CSimplePointsMap& m2, CPose2D& initPose, CPose2D& outPose)
{
	float runTime;
	CPosePDFPtr pdf = m_pICP->Align(
			&m1,
			&m2,
			initPose,
			&runTime,
			(void*)(getRet())
			);
	// cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;

	*m_pPose = CPose2D(pdf->getMeanVal());
	m_pTrans->copyFrom(*pdf);
	/*if(!skip_window){
		CPosePDFGaussian  gPdf;
		gPdf.copyFrom(*pdf);
		cout << "Covariance of estimation: " << endl << gPdf.cov << endl;
		cout << " std(x): " << sqrt( gPdf.cov(0,0) ) << endl;
		cout << " std(y): " << sqrt( gPdf.cov(1,1) ) << endl;
		cout << " std(phi): " << RAD2DEG(sqrt( gPdf.cov(2,2) )) << " (deg)" << endl;
	}*/
	int iter; 
	float good;
	float quality;
	getMatchQuality(iter,good,quality);
	outPose = *m_pPose;

	// If we have 2D windows, use'em:
#if MRPT_HAS_WXWIDGETS
	if (!skip_window)
	{
		cout<<"iter: "<<iter<<" good: "<<good<<" quality: "<<quality    <<endl;
		CSimplePointsMap m2_trans = m2;
		CMatrixFloat COV22 =  CMatrixFloat( CMatrixDouble( m_pTrans->cov ));
		COV22.setSize(2,2);
		CVectorFloat MEAN2D(2);
		MEAN2D[0] = m_pTrans->mean.x();
		MEAN2D[1] = m_pTrans->mean.y();
		// cout<<"ZHICP.cpp: mean: "<<m_pTrans->mean<<endl;
		m2_trans.changeCoordinatesReference(m_pTrans->mean);
		gui::CDisplayWindowPlots	win("ICP results");

		// Reference map:
		vector_float   map1_xs, map1_ys, map1_zs;
		m1.getAllPoints(map1_xs,map1_ys,map1_zs);
		win.plot( map1_xs, map1_ys, "b.3", "map1");

		// Translated map:
		vector_float   map2_xs, map2_ys, map2_zs;
		m2_trans.getAllPoints(map2_xs,map2_ys,map2_zs);
		win.plot( map2_xs, map2_ys, "r.3", "map2");

		// Uncertainty
		win.plotEllipse(MEAN2D[0],MEAN2D[1],COV22,3.0,"b2", "cov");

		win.axis(-1,10,-6,6);
		win.axis_equal();

		cout << "Close the window to exit" << endl;
		win.waitForKey();
	}
#endif
	return good;
}

float CICPWarpper::ICPMatch(float* rx, float* ry, float* ax, float* ay, int N, double pini[3], double pout[3])
{
	CSimplePointsMap m1,m2;
	// CICP::TReturnInfo info;
	m1.resize(N);
	m2.resize(N);
	for(int i=0;i<N;i++)
		m1.setPointFast(i,rx[i],ry[i],0);
	for(int i=0;i<N;i++)
		m2.setPointFast(i,ax[i],ay[i],0);
	CPose2D initPose(pini[0],pini[1],pini[2]);
	CPose2D outPose;
	float goodness = ICP(m1,m2,initPose,outPose);
	pout[0] = outPose[0]; 
	pout[1] = outPose[1];
	pout[2] = outPose[2];
	return goodness;
}

void CICPWarpper::ICPMatch(vector<float>& rx, vector<float>& ry, vector<float>& ax, vector<float>& ay, double px, double py, double pth)
{
	CSimplePointsMap m1,m2;
	// CICP::TReturnInfo info;
	float runTime;
	m1.resize(rx.size());
	m2.resize(ax.size());
	for(int i=0;i<rx.size();i++)
		m1.setPointFast(i,rx[i],ry[i],0);
	for(int i=0;i<ax.size();i++)
		m2.setPointFast(i,ax[i],ay[i],0);
	CPose2D initPose(px,py,pth);
	CPose2D outPose;
	ICP(m1,m2, initPose, outPose);
/*
	CPosePDFPtr pdf = m_pICP->Align(
			&m1,
			&m2,
			initPose,
			&runTime,
			(void*)(getRet())
			);
	// cout << "Mean of estimation: " << pdf->getMeanVal() << endl<< endl;

	*m_pPose = CPose2D(pdf->getMeanVal());
	m_pTrans->copyFrom(*pdf);
	if(!skip_window){
		CPosePDFGaussian  gPdf;
		gPdf.copyFrom(*pdf);
		cout << "Covariance of estimation: " << endl << gPdf.cov << endl;
		cout << " std(x): " << sqrt( gPdf.cov(0,0) ) << endl;
		cout << " std(y): " << sqrt( gPdf.cov(1,1) ) << endl;
		cout << " std(phi): " << RAD2DEG(sqrt( gPdf.cov(2,2) )) << " (deg)" << endl;
	}
	// If we have 2D windows, use'em:
#if MRPT_HAS_WXWIDGETS
	int iter; 
	float good;
	float quality;
	if (!skip_window)
	{
		getMatchQuality(iter,good,quality);
		cout<<"iter: "<<iter<<" good: "<<good<<" quality: "<<quality    <<endl;
		CSimplePointsMap m2_trans = m2;
		CMatrixFloat COV22 =  CMatrixFloat( CMatrixDouble( m_pTrans->cov ));
		COV22.setSize(2,2);
		CVectorFloat MEAN2D(2);
		MEAN2D[0] = m_pTrans->mean.x();
		MEAN2D[1] = m_pTrans->mean.y();
		m2_trans.changeCoordinatesReference(m_pTrans->mean);
		gui::CDisplayWindowPlots	win("ICP results");

		// Reference map:
		vector_float   map1_xs, map1_ys, map1_zs;
		m1.getAllPoints(map1_xs,map1_ys,map1_zs);
		win.plot( map1_xs, map1_ys, "b.3", "map1");

		// Translated map:
		vector_float   map2_xs, map2_ys, map2_zs;
		m2_trans.getAllPoints(map2_xs,map2_ys,map2_zs);
		win.plot( map2_xs, map2_ys, "r.3", "map2");

		// Uncertainty
		win.plotEllipse(MEAN2D[0],MEAN2D[1],COV22,3.0,"b2", "cov");

		win.axis(-1,10,-6,6);
		win.axis_equal();

		cout << "Close the window to exit" << endl;
		win.waitForKey();
	}
#endif
*/
}

void CICPWarpper::getResult(double p[3], double cov[6])
{
	p[0] = (*m_pPose)[0]; p[1] = (*m_pPose)[1]; p[2] = (*m_pPose)[2];
	cov[0] = m_pTrans->cov(0,0); cov[1] = m_pTrans->cov(0,1); cov[2] = m_pTrans->cov(0,2);
				     cov[3] = m_pTrans->cov(1,1); cov[4] = m_pTrans->cov(1,2);
				     				  cov[5] = m_pTrans->cov(2,2);
}
void CICPWarpper::getMatchQuality(int& iterations, float& good, float& quality)
{
	iterations = getRet()->nIterations;
	good = getRet()->goodness;
	quality = getRet()->quality;
}

