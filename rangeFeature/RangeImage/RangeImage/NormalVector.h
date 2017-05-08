#ifndef NORMALVECTOR_H
#define NORMALVECTOR_H

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

class CNormalVector
{
public:
	CNormalVector();
	CNormalVector(float x,float y, float z);
	template<typename PointT>
	CNormalVector(const PointT& p1,const PointT& p2, const PointT& p3)
	{
		CalculateNV(p1,p2,p3);
	}
	~CNormalVector();
	void Normalization();
	void GetNormal(float& x,float& y,float& z);
	
	template<typename PT>
	double DotProduct(PT& pt)
	{
		return (nx*pt.x+ny*pt.y+nz*pt.z);
	}
	
	template<>
	double DotProduct(CNormalVector& otherNV){
		return (nx*otherNV.nx+ny*otherNV.ny+nz*otherNV.nz);
	}

public:
	float nx,ny,nz;
public:
	template<typename PointT>
	void CalculateNV(const PointT& a,const PointT& b, const PointT& c)
	{
		float f1x = b.x - a.x;
		float f1y = b.y - a.y;
		float f1z = b.z - a.z;
		float f2x = c.x - a.x;
		float f2y = c.y - a.y;
		float f2z = c.z - a.z;
		nx = f1y*f2z - f1z*f2y;
		ny = -(f1x*f2z - f1z*f2x);
		nz = f1x*f2y - f1y*f2x;
		Normalization();
	}
};

#endif