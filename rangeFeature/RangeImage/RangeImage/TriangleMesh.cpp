#include "TriangleMesh.h"
#include <pcl/filters/voxel_grid.h>
#include "boost/dynamic_bitset.hpp"

// Parameters for filtering small polygons
int CTriangleMesh::m_filter_threshold_for_small_polygon=3;

CTriangleMesh::CTriangleMesh():m_pVertex(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pTriSet(new TriangleSet)
{}
CTriangleMesh::CTriangleMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _pPC):m_pVertex(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pTriSet(new TriangleSet)
{
	InitVertexes(_pPC);
}
CTriangleMesh::~CTriangleMesh(){}

void CTriangleMesh::compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_input,\
			 boost::shared_ptr<Triangles>& m_trianlges)
{
	InitVertexes(m_input);
	FromPt2Cell();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tri_pc(m_pVertex->makeShared());
	FromCell2TriMesh();
	
	m_trianlges->m_cloud=tri_pc;
	vector<boost::shared_ptr<TriangleIndex> >& m_pTrindex=m_pTriSet->m_vtindex;
	vector<int> tri_index;
	tri_index.resize(m_pTrindex.size()*3);
	for(size_t i=0,j=0;i<m_pTrindex.size();i++)
	{
		j=i*3;
		tri_index[j]=m_pTrindex[i]->m_tindex[0];
		tri_index[j+1]=m_pTrindex[i]->m_tindex[1];
		tri_index[j+2]=m_pTrindex[i]->m_tindex[2];
	}
	m_trianlges->m_indices.swap(tri_index);
}

// delete infinite vertexes
void CTriangleMesh::InitVertexes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _pPC){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	tmpPC->points.resize(_pPC->points.size());
	int j=0;
	for(size_t i=0;i<_pPC->points.size();i++)
	{
		pcl::PointXYZRGB& sp=_pPC->points[i];
		if(_isnan(sp.x) || _isnan(sp.y) || _isnan(sp.z)){
			continue;
		}
		tmpPC->points[j]=sp;
		j++;
	}
	tmpPC->points.resize(j);

	cout<<"Before filter, N of points: "<<tmpPC->points.size()<<endl;
	// Create the filtering object
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (tmpPC);
	sor.setLeafSize (0.02f, 0.02f, 0.02f);
	sor.filter (*m_pVertex);
	cout<<"After filter, N of points: "<<m_pVertex->points.size()<<endl;
//	m_pVertex.swap(tmpPC);
}

bool CTriangleMesh::FromPt2Cell()
{
	if(m_pVertex->points.size()==0)
	{
		cout<<"no points in CTriangleMesh!"<<endl;
		return false;
	}
	// clear all cell points and index
	m_CellIndex.clear(); // indexes of cell
	m_CellSet.clear();	// links

	int index_of_cell=m_CellSet.size();
	for(size_t i=0;i<m_pVertex->points.size();i++)
	{
		pcl::PointXYZRGB& sp=m_pVertex->points[i];
		IPt InterPt=get_index_point(sp);
		if(InterPt.iy==Y_IMPOSSIBLE)
			continue;
		if(m_CellIndex.find(InterPt)!=m_CellIndex.end())// this cell has been taken!
			continue;
		m_CellIndex.insert(make_pair(InterPt,index_of_cell));
		m_CellSet.push_back(boost::shared_ptr<CVtx>(new CVtx(i)));
		AdjustCrossLinks(i,InterPt);
		index_of_cell++;
	}
}
void CTriangleMesh::AdjustCrossLinks(int index_pt,IPt index_cell)
{
	IPt tmp=index_cell;
	map<IPt,int>::iterator it_self=m_CellIndex.find(index_cell);
	boost::shared_ptr<CVtx>& rself=m_CellSet[it_self->second];
	map<IPt,int>::iterator it_cell;
	// adjust before cell
	tmp.iz-=CELL_STEP;
	if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
	{
		boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
		tmpCell->m_behind_pt=rself->m_self_pt;
		rself->m_before_pt=tmpCell->m_self_pt;	
		//m_CellSet[it_cell->second]->m_behind_pt=index_pt;
		//m_CellSet[it_self->second]->m_before_pt=m_CellSet[it_cell->second]->m_self_pt;
	}
	else
	{
		tmp.iz-=CELL_STEP;
		if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
		{
			boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
			tmpCell->m_behind_pt=rself->m_self_pt;
			rself->m_before_pt=tmpCell->m_self_pt;
		}
		tmp.iz+=CELL_STEP;
	}
	tmp.iz+=CELL_STEP;
	// adjust behind cell
	tmp.iz+=CELL_STEP;
	if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
	{
		boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
		tmpCell->m_before_pt=rself->m_self_pt;
		rself->m_behind_pt=tmpCell->m_self_pt;	
	}
	else
	{
		tmp.iz+=CELL_STEP;
		if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
		{
			boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
			tmpCell->m_before_pt=rself->m_self_pt;
			rself->m_behind_pt=tmpCell->m_self_pt;	
		}
		tmp.iz-=CELL_STEP;
	}
	tmp.iz-=CELL_STEP;
	// adjust left cell
	tmp.ix-=CELL_STEP;
	if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
	{
		boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
		tmpCell->m_right_pt=rself->m_self_pt;
		rself->m_left_pt=tmpCell->m_self_pt;
	}
	else
	{
		tmp.ix-=CELL_STEP;
		if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
		{
			boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
			tmpCell->m_right_pt=rself->m_self_pt;
			rself->m_left_pt=tmpCell->m_self_pt;
		}
		tmp.ix+=CELL_STEP;
	}
	tmp.ix+=CELL_STEP;
	// adjust right cell
	tmp.ix+=CELL_STEP;
	if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
	{
		boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
		tmpCell->m_left_pt=rself->m_self_pt;
		rself->m_right_pt=tmpCell->m_self_pt;
	}
	else
	{
		tmp.ix+=CELL_STEP;
		if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
		{
			boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
			tmpCell->m_left_pt=rself->m_self_pt;
			rself->m_right_pt=tmpCell->m_self_pt;
		}
		tmp.ix-=CELL_STEP;
	}
	tmp.ix-=CELL_STEP;
	// adjust upper cell
	tmp.iy-=CELL_STEP;
	if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
	{
		boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
		tmpCell->m_lower_pt=rself->m_self_pt;
		rself->m_upper_pt=tmpCell->m_self_pt;
	}
	else
	{
		tmp.iy-=CELL_STEP;
		if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
		{
			boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
			tmpCell->m_lower_pt=rself->m_self_pt;
			rself->m_upper_pt=tmpCell->m_self_pt;
		}
		tmp.iy+=CELL_STEP;
	}
	tmp.iy+=CELL_STEP;
	// adjust lower cell
	tmp.iy+=CELL_STEP;
	if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
	{
		boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
		tmpCell->m_upper_pt=rself->m_self_pt;
		rself->m_lower_pt=tmpCell->m_self_pt;
	}
	else
	{
		tmp.iy+=CELL_STEP;
		if((it_cell=m_CellIndex.find(tmp))!=m_CellIndex.end())
		{
			boost::shared_ptr<CVtx>& tmpCell=m_CellSet[it_cell->second];
			tmpCell->m_upper_pt=rself->m_self_pt;
			rself->m_lower_pt=tmpCell->m_self_pt;
		}
		tmp.iy-=CELL_STEP;
	}
	tmp.iy-=CELL_STEP;
}

void CTriangleMesh::GetVertex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOut)
{
	int index_of_vertex=0;
	for(size_t i=0;i<m_CellSet.size();i++)
	{
		index_of_vertex=m_CellSet[i]->m_self_pt;
		pcl::PointXYZRGB& sp=m_pVertex->points[index_of_vertex];
		pOut->points.push_back(sp);
	}	
	cout<<"size in m_CellSet is: "<<m_CellSet.size()<<endl;
}

void CTriangleMesh::GetOutlierTriIndices(vector<pcl::Vertices>& rIndices,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOut)
{
	int v[3];
#define MAX_PT_DIS 0.175 // 1dm
	for(size_t i=0;i<rIndices.size();i++)
	{
		pcl::Vertices& rVertex=rIndices[i];
		for(int j=0;j<3;j++)
			v[j]=rVertex.vertices[j];
		pcl::PointXYZRGB& sp1=m_pVertex->points[v[0]];
		pcl::PointXYZRGB& sp2=m_pVertex->points[v[1]];
		pcl::PointXYZRGB& sp3=m_pVertex->points[v[2]];

		if(getdis(sp1,sp2)>MAX_PT_DIS || getdis(sp1,sp3)>MAX_PT_DIS || getdis(sp2,sp3)>MAX_PT_DIS )
		{
			disloc(sp1);		pOut->points.push_back(sp1);
			disloc(sp2);		pOut->points.push_back(sp2);
			disloc(sp3,true);	pOut->points.push_back(sp3);
			discellIndex(sp1);
			discellIndex(sp2);
			discellIndex(sp3,true);
		}
	}	
}

void CTriangleMesh::GetOutlierVertex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOut)
{
	int index_of_vertex=0;
	int v[3];
#define MAX_PT_DIS 0.175 // 1dm
	for(size_t i=0;i<m_pTriSet->m_vtindex.size();i++)
	{
		boost::shared_ptr<TriangleIndex>& rTriIndex=m_pTriSet->m_vtindex[i];
		rTriIndex->getVertexes(v);
		pcl::PointXYZRGB& sp1=m_pVertex->points[v[0]];
		pcl::PointXYZRGB& sp2=m_pVertex->points[v[1]];
		pcl::PointXYZRGB& sp3=m_pVertex->points[v[2]];

		if(getdis(sp1,sp2)>MAX_PT_DIS || getdis(sp1,sp3)>MAX_PT_DIS || getdis(sp2,sp3)>MAX_PT_DIS )
		{
			disloc(sp1);		pOut->points.push_back(sp1);
			disloc(sp2);		pOut->points.push_back(sp2);
			disloc(sp3,true);	pOut->points.push_back(sp3);
			discellIndex(sp1);
			discellIndex(sp2);
			discellIndex(sp3,true);
		}
		
	}	

}
// Generate TriangleMesh depending on Cells
void CTriangleMesh::FromCell2TriMesh()
{
	for(size_t i=0;i<m_CellSet.size();i++)
	{
		boost::shared_ptr<CVtx>& rCV=m_CellSet[i];
		// find left-down triangle
		if(rCV->m_left_pt!=-1 && rCV->m_lower_pt!=-1){
			m_pTriSet->addtriangle(TriangleIndex(rCV->m_self_pt,rCV->m_left_pt,rCV->m_lower_pt),m_pVertex);
		}
		// find right-up triangle
		if(rCV->m_right_pt!=-1 && rCV->m_upper_pt!=-1){
			m_pTriSet->addtriangle(TriangleIndex(rCV->m_self_pt,rCV->m_right_pt,rCV->m_upper_pt),m_pVertex);
		}
		// find before-down triangle
		if(rCV->m_before_pt!=-1 && rCV->m_lower_pt!=-1){
			m_pTriSet->addtriangle(TriangleIndex(rCV->m_self_pt,rCV->m_before_pt,rCV->m_lower_pt),m_pVertex);
		}
		// find behind-upper triangle
		if(rCV->m_behind_pt!=-1 && rCV->m_upper_pt!=-1){
			m_pTriSet->addtriangle(TriangleIndex(rCV->m_self_pt,rCV->m_behind_pt,rCV->m_upper_pt),m_pVertex);
		}	
		// find left-before triangle
		if(rCV->m_left_pt!=-1 && rCV->m_before_pt!=-1){
			m_pTriSet->addtriangle(TriangleIndex(rCV->m_self_pt,rCV->m_left_pt,rCV->m_before_pt),m_pVertex);
		}
		// find right-behind triangle
		if(rCV->m_right_pt!=-1 && rCV->m_behind_pt!=-1){
			m_pTriSet->addtriangle(TriangleIndex(rCV->m_self_pt,rCV->m_right_pt,rCV->m_behind_pt),m_pVertex);
		}
	}
}

void CTriangleMesh::GetTriIndices(vector<pcl::Vertices>& rIndices)
{
	boost::shared_ptr<TriangleIndex>& rtri=m_pTriSet->m_vtindex[0];
	pcl::Vertices pclV;
	pclV.vertices.resize(3);
	int v[3];
	for(size_t i=0;i<m_pTriSet->m_vtindex.size();i++)
	{
		rtri=m_pTriSet->m_vtindex[i];
		rtri->getVertexes(v);
		for(int j=0;j<3;j++)
			pclV.vertices[j]=v[j];
		rIndices.push_back(pclV);
	}
}

#define COS10 0.98480775301220805936674302458952
#define COS20 0.93969262078590838405410927732473
#define COS50 0.64278760968653932632264340990726
#define THRE COS20

// Merge near triangles into one set (Polygon)
// Then delete those small enough
void CTriangleMesh::FilterSmallPolygon()
{

	if(m_pTriSet->size()<=0)
	{
		cout<<"No triangles in CTriangleMesh!"<<endl;
		return;
	}

	// each triangle must be used once
	boost::dynamic_bitset<> UnUsed;
	UnUsed.resize(m_pTriSet->size(),1);

	// tmp polygon struct
	vector< vector<boost::shared_ptr<TriangleIndex> > > polygons;
	vector<boost::shared_ptr<TriangleIndex> > single_polygon;

	while(UnUsed.any()/*UnUsed.count()!=UnUsed.size()*/){
		// clear single_polygon
		single_polygon.clear();
		// index of all triangles
		queue<int> index_tri;
		size_t first_Unused=UnUsed.find_first();
		index_tri.push(first_Unused);
		int cur_index;
		while(!index_tri.empty())
		{
			cur_index=index_tri.front();
			index_tri.pop();
			UnUsed[cur_index]=false;
			single_polygon.push_back(m_pTriSet->m_vtindex[cur_index]);	
			
			// check three edges around this triangle
			boost::shared_ptr<TriangleIndex>& ptri=m_pTriSet->m_vtindex[cur_index];
			boost::shared_ptr<CNormalVector>& pnv=m_pTriSet->m_nv[cur_index];

			int v1,v2,v3;
			ptri->getVertexes(v1,v2,v3);
			set<TriangleEdge >::iterator it_edge;
			int edge_index;
			
			// edge (v1,v2)
			it_edge=m_pTriSet->m_edges.find(TriangleEdge(v1,v2));
			if(it_edge==m_pTriSet->m_edges.end())
			{
				cout<<"error in FilterSmallPolygon()!"<<endl;
				return;
			}
			for(size_t k=0;k<it_edge->m_ptindex->size();k++)
			{
				if(!UnUsed[(*(it_edge->m_ptindex))[k]]) continue;
				edge_index=(*(it_edge->m_ptindex))[k];
				boost::shared_ptr<CNormalVector>& pnv1=m_pTriSet->m_nv[edge_index];
				if(pnv->DotProduct(*pnv1) >= THRE )
				{
					index_tri.push(edge_index);
					UnUsed[edge_index]=false;
				}
			}
			
			// edge (v1,v3)
			it_edge=m_pTriSet->m_edges.find(TriangleEdge(v1,v3));
			if(it_edge==m_pTriSet->m_edges.end())
			{
				cout<<"error in FilterSmallPolygon()!"<<endl;
				return;
			}
			for(size_t k=0;k<it_edge->m_ptindex->size();k++)
			{
				if(!UnUsed[(*(it_edge->m_ptindex))[k]]) continue;
				edge_index=(*(it_edge->m_ptindex))[k];
				boost::shared_ptr<CNormalVector>& pnv1=m_pTriSet->m_nv[edge_index];
				if(pnv->DotProduct(*pnv1) >= THRE )
				{
					index_tri.push(edge_index);
					UnUsed[edge_index]=false;
				}
			}
			// edge (v2,v3)
			it_edge=m_pTriSet->m_edges.find(TriangleEdge(v1,v2));
			if(it_edge==m_pTriSet->m_edges.end())
			{
				cout<<"error in FilterSmallPolygon()!"<<endl;
				return;
			}
			for(size_t k=0;k<it_edge->m_ptindex->size();k++)
			{
				if(!UnUsed[(*(it_edge->m_ptindex))[k]]) continue;
				edge_index=(*(it_edge->m_ptindex))[k];
				boost::shared_ptr<CNormalVector>& pnv1=m_pTriSet->m_nv[edge_index];
				if(pnv->DotProduct(*pnv1) >= THRE )
				{
					index_tri.push(edge_index);
					UnUsed[edge_index]=false;
				}
			}

		}
		polygons.push_back(single_polygon);
	}
	
	/*for(size_t i=0;i<polygons.size();i++){
		cout<<"polygon "<<i+1<<" has "<<polygons[i].size()<<" triangles!"<<endl;
	}*/

	// filtering those triangles in small polygons
	for(size_t i=0;i<polygons.size();i++)
	{
		if(polygons[i].size()>=CTriangleMesh::m_filter_threshold_for_small_polygon)
		{
			m_pTriSet->m_vtindex_backup.insert(m_pTriSet->m_vtindex_backup.end(),polygons[i].begin(),polygons[i].end());
		}
	}
	m_pTriSet->m_vtindex_backup.swap(m_pTriSet->m_vtindex);
}