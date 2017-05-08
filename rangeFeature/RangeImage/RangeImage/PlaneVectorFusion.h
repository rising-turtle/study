#ifndef PLANE_VECTOR_FUSION_H
#define PLANE_VECTOR_FUSION_H

#include "preheader.h"
#include "globaldefinition.h"
#include "VoxelGridAndNormal.h"

class CPlaneVectorFusion{
	typedef boost::shared_ptr<CSurfaceInfo> SurfacePTR;
	typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> > PointPTR;
public:
	CPlaneVectorFusion();
	~CPlaneVectorFusion();
	
	// �ں϶��ƽ����Ҫ
	vector<SurfacePTR> m_surface;
	vector<PointPTR>  m_pcs;
	vector<bool> m_isplane;
public:
	// ����һ������������
	void AddPlaneVectors(CVoxelGridAndNormal* pPlaneV);
	void AddNoisyPatch(CVoxelGridAndNormal* pPlaneV, int index);
	// �Ƿ��Ƿǳ����Ƶ���
	bool IsSimilarPlane(SurfacePTR& p1, SurfacePTR& p2);
	void BlendWithPlane(CVoxelGridAndNormal* pPlaneV, int this_id, int that_id);
	void AddNewPatch(CVoxelGridAndNormal* pPlaneV, int index);
	// �������ϵĵ��λ��
	void AdjustPTs();
	void GetAllPoints(PointPTR& out_pc, bool only_plane=false);
};


#endif