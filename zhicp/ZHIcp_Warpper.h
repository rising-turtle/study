#ifndef ZHICP_WARPPER_H
#define ZHICP_WARPPER_H

#include <vector>

using namespace std;
namespace mrpt{
	namespace slam{
		class CICP;
		class CSimplePointsMap;
		// struct TReturnInfo;
	}
	namespace poses{
		class CPosePDFGaussian;
		class CPose2D;
	}
}
// class mrpt::slam::CICP;
// class mrpt::poses::CPosePDFGaussian;
class CICPWarpper {
public:
	CICPWarpper();
	~CICPWarpper();
	void initICP();
	float ICP(mrpt::slam::CSimplePointsMap&, mrpt::slam::CSimplePointsMap&, mrpt::poses::CPose2D&, mrpt::poses::CPose2D&);
	float ICPMatch(float*, float*, float*, float*, int,\
			double pini[3], double pout[3]);
	void ICPMatch(vector<float>& rx, vector<float>& ry,vector<float>& ax,vector<float>& ay, double,double,double);
	void getResult(double p[3], double cov[6]);
	void getMatchQuality(int&, float&, float&);
public:
	bool skip_window;
	int ICP_method ;
	mrpt::slam::CICP * m_pICP;
	mrpt::poses::CPosePDFGaussian * m_pTrans;
	mrpt::poses::CPose2D * m_pPose;
	// struct mrpt::slam::TReturnInfo * m_pRet;
};

#endif
