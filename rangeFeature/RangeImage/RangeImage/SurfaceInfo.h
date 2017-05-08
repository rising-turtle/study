#ifndef SURFACEINFO_H
#define SURFACEINFO_H
#include "preheader.h"
#include "NormalVector.h"

typedef enum{CUPBOARD,WALL,FLOOR,TABLE,UNKNOWN} PLANE_TYPE;

class CSurfaceInfo{
public:
	CSurfaceInfo():m_nx(0),m_ny(0),m_nz(0),m_num(0),m_D(0){
		for(int i=0;i<4;i++){
			m_total.data[i]=m_centorid.data[i]=0;
		}
		m_bD=false;
	}
	~CSurfaceInfo(){}
public:
	void Blend(const CSurfaceInfo& other);
	void Reset();
public:
	inline void AddPtAndNormal(pcl::PointXYZRGB& pt, pcl::Normal& nv)
	{
		m_nx+=nv.normal_x; m_ny+=nv.normal_y; m_nz+=nv.normal_z;
		m_num++;
		for(int i=0;i<4;i++)
			m_total.data[i]+=pt.data[i];
	}
	inline void ComputeD(){
		m_D=m_nv.nx*m_centorid.x+m_nv.ny*m_centorid.y+m_nv.nz*m_centorid.z;
		m_D*=-1.0;
		m_bD=true;
	}
	inline void Compute(){
		if(m_num<=0)
		{
			cout<<"error in Compute!"<<endl;
			return ;
		}
		float factor=1.0/(float)m_num;
		m_nv.nx=m_nx*factor; 
		m_nv.ny=m_ny*factor;
		m_nv.nz=m_nz*factor;
		m_nv.Normalization();
		for(int i=0;i<4;i++)
			m_centorid.data[i]=m_total.data[i]*factor;
	}
	template<typename PT>
	void ComputeZ(PT& pt) // 计算点pt到平面投影
	{
		float tmp1=m_nv.nx*m_centorid.x+m_nv.ny*m_centorid.y+m_nv.nz*m_centorid.z;
		float tmp2=pt.x*m_nv.nx+pt.y*m_nv.ny+pt.z*m_nv.nz;
		float t = tmp2-tmp1;
		pt.x=pt.x-m_nv.nx*t;
		pt.y=pt.y-m_nv.ny*t;
		pt.z=pt.z-m_nv.nz*t;
	}
	template<typename PT>
	float point2plane(PT& pt) // 计算点到平面的距离
	{
		if(!m_bD)
			ComputeD();
		return fabs(m_nv.DotProduct(pt)+m_D);
	}
public:
	// total sum 
	float m_nx;
	float m_ny;
	float m_nz;
	size_t m_num;				// 包含点数
	float m_D;					// 平面方程 AX+BY+CZ+D=0 的D
	bool m_bD;
	CNormalVector m_nv;			// 法向量信息
	pcl::PointXYZ m_total;		
	pcl::PointXYZ m_centorid;	// 重心点
};

#endif