#include "PlaneVectorFusion.h"

#define COS10 0.98480775301220805936674302458952

CPlaneVectorFusion::CPlaneVectorFusion(){}
CPlaneVectorFusion::~CPlaneVectorFusion(){}

void CPlaneVectorFusion::AddNoisyPatch(CVoxelGridAndNormal* pPlaneV, int index)
{
	if(index<0 || index>= pPlaneV->m_surfaces.size()){
		cout<<"error in AddNoisyPatch!"<<endl;
		return ;
	}
	m_surface.push_back(pPlaneV->m_surfInfo[index]);	
	m_isplane.push_back(false);
	PointPTR m_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i=0;i<pPlaneV->m_surfaces[index].size();i++){
		m_pc->points.push_back(pPlaneV->m_pFilterPC->points[pPlaneV->m_surfaces[index][i]]);
	}
	m_pcs.push_back(m_pc);
}

// 是否是非常相似的面
bool CPlaneVectorFusion::IsSimilarPlane(SurfacePTR& p1, SurfacePTR& p2)
{
	double angle=p1->m_nv.DotProduct(p2->m_nv);
	if(angle<=COS10)
	{
		return false;
	}

	// 点到面的距离不能太大
	double dis = (p1->point2plane(p2->m_centorid)+p2->point2plane(p1->m_centorid))/2;
	if(dis >= 0.2) 
		return false;

	// 对于地面则不用判断相邻性
	if(fabs(p1->m_nv.ny)>0.9)
	{
		return true;
	}

	// TODO 两个面在法向量投影上有较大重叠
	


	// 两个面必须相邻
	pcl::PointXYZRGB pt1,pt2;
	pt1.x=p1->m_centorid.x;
	pt1.y=p1->m_centorid.y;
	pt1.z=p1->m_centorid.z;
	pt2.x=p2->m_centorid.x;
	pt2.y=p2->m_centorid.y;
	pt2.z=p2->m_centorid.z;
	//if(!((CVoxelGridAndNormal*)0)->IsAdjacent(pt1,pt2))
		//return false;
	
	return true;

}


void CPlaneVectorFusion::AddNewPatch(CVoxelGridAndNormal* pPlaneV, int index)
{
	m_surface.push_back(pPlaneV->m_surfInfo[index]);
	m_isplane.push_back(true);
	PointPTR tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	for(int i=0;i<pPlaneV->m_surfaces[index].size();i++){
		tmpPC->points.push_back(pPlaneV->m_pFilterPC->points[pPlaneV->m_surfaces[index][i]]);
	}
	m_pcs.push_back(tmpPC);
}

void CPlaneVectorFusion::BlendWithPlane(CVoxelGridAndNormal* pPlaneV, int this_id, int that_id)
{
	SurfacePTR& pcur = pPlaneV->m_surfInfo[that_id];
	SurfacePTR& ppre = m_surface[this_id];
	PointPTR& pc = m_pcs[this_id];
	ppre->Blend(*pcur);
	for(int i=0;i<pPlaneV->m_surfaces[that_id].size();i++)
		pc->points.push_back(pPlaneV->m_pFilterPC->points[pPlaneV->m_surfaces[that_id][i]]);
}

// 加入一个面向量集合
void CPlaneVectorFusion::AddPlaneVectors(CVoxelGridAndNormal* pPlaneV)
{
	// 遍历向量集里面所有的patch，如果是面，尽量融合
	vector<int> index_set;
	for(int i=0;i<pPlaneV->m_surfaces.size();i++){
		SurfacePTR& pcur = pPlaneV->m_surfInfo[i];
		if(!pPlaneV->m_plane_index[i]){ // 不是平面
			AddNoisyPatch(pPlaneV,i);
			continue;
		}
		// 是平面，搜索是否已经有相似的面
		bool IsFused=false;
		for(int j=0;j<m_surface.size();j++){
			if(!m_isplane[j])
				continue;
			SurfacePTR& ppre = m_surface[j];
			if(IsSimilarPlane(ppre,pcur)){ // 如果相似
				BlendWithPlane(pPlaneV,j,i);
				IsFused=true;
				break;
			}
		}
		if(!IsFused){  // 没有相似的，就加为新的面
			//AddNewPatch(pPlaneV,i);
			index_set.push_back(i);
		}
	}
	for(int i=0;i<index_set.size();i++)
		AddNewPatch(pPlaneV,index_set[i]);
}

// 调整面上的点的位置
void CPlaneVectorFusion::AdjustPTs()
{
	for(int i=0;i<m_surface.size();i++){
		if(m_isplane[i]){
			SurfacePTR& psur=m_surface[i];
			for(int j=0;j<m_pcs[i]->points.size();j++){
				psur->ComputeZ(m_pcs[i]->points[j]);
			}
		}

	}
}

void CPlaneVectorFusion::GetAllPoints(PointPTR& out_pc, bool only_plane)
{
	for(int i=0;i<m_surface.size();i++){
		if(!only_plane || m_isplane[i]){
			out_pc->points.insert(out_pc->points.end(),m_pcs[i]->points.begin(),m_pcs[i]->points.end());
		}
	}
}