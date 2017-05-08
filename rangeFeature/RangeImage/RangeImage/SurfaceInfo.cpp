#include "SurfaceInfo.h"

void CSurfaceInfo::Blend(const CSurfaceInfo& other)
{
	size_t total=m_num+other.m_num;
	float factor1=(float)(m_num)/(float)(total);
	float factor2=1-factor1;
	m_num=total;
	
	m_nx+=other.m_nx;
	m_ny+=other.m_ny;
	m_nz+=other.m_nz;

	for(int i=0;i<4;i++)
		m_total.data[i]+=other.m_total.data[i];

	// normal vector
	m_nv.nx=m_nv.nx*factor1+other.m_nv.nx*factor2;
	m_nv.ny=m_nv.ny*factor1+other.m_nv.ny*factor2;
	m_nv.nz=m_nv.nz*factor1+other.m_nv.nz*factor2;
	m_nv.Normalization();

	// centorid point
	m_centorid.x=m_centorid.x*factor1+other.m_centorid.x*factor2;
	m_centorid.y=m_centorid.y*factor1+other.m_centorid.y*factor2;
	m_centorid.z=m_centorid.z*factor1+other.m_centorid.z*factor2;
}
void CSurfaceInfo::Reset()
{
	m_nx=0; m_ny=0; m_nz=0;
	m_num=0; m_D=0;
	for(int i=0;i<4;i++){
		m_total.data[i]=m_centorid.data[i]=0;
	}
	m_bD=false;
}
