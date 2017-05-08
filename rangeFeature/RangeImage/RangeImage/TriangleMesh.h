#ifndef TRIANGLE_MESH_H
#define TRIANGLE_MESH_H

#include "preheader.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/Vertices.h"
#include "boost/shared_ptr.hpp"
#include "NormalVector.h"

#define Y_IMPOSSIBLE -100
#define CELL_STEP 1 // 4cm for each cell

typedef struct _triangle{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
	vector<int> m_indices;
	string m_name;
}Triangles;

// identify Internal Cell index of each point
class IPt{
public:
	IPt(int _ix=-1,int _iy=Y_IMPOSSIBLE,int _iz=-1):ix(_ix),iy(_iy),iz(_iz){}
	int ix,iy,iz; // index of each cell
	bool operator <(const IPt& other) const{
		return ( ix < other.ix || (ix==other.ix && iy<other.iy) || (ix==other.ix && iy==other.iy && iz<other.iz));
	}
};
// like double cross-links 
class CVtx{
public:
	CVtx(int _self_index=-1):m_self_pt(_self_index){
		m_before_pt=m_behind_pt=m_left_pt=m_right_pt=m_upper_pt=m_lower_pt=-1;
	}
public:
	int m_self_pt; // index to self point 
	int m_before_pt; // index to before point
	int m_behind_pt; // index to behind point
	int m_left_pt; // index to left point
	int m_right_pt; // index to right point
	int m_upper_pt; // index to upper point
	int m_lower_pt; // index to lower point
};
class TriangleIndex{
public:
	TriangleIndex(int v1,int v2, int v3){m_tindex[0]=v1;m_tindex[1]=v2;m_tindex[2]=v3;}
	TriangleIndex(const TriangleIndex& o){m_tindex[0]=o.m_tindex[0];m_tindex[1]=o.m_tindex[1];m_tindex[2]=o.m_tindex[2];}
	inline void getVertexes(int &v1,int& v2,int& v3) const {v1=m_tindex[0];v2=m_tindex[1];v3=m_tindex[2];}
	inline void getVertexes(int v[3]) const {getVertexes(v[0],v[1],v[2]);}
public:
	int m_tindex[3];
};

class TriangleEdge{
public:
	TriangleEdge(int v1, int v2):m_ptindex(new vector<int> ){m_endp[0]=v1<v2?v1:v2; m_endp[1]=v1<v2?v2:v1;}
	void addtriangle(int tindex){
		m_ptindex->push_back(tindex);
	}
	bool operator<(const TriangleEdge& o) const{
		return ( m_endp[0]< o.m_endp[0] || (m_endp[0] == o.m_endp[0] && m_endp[1]<o.m_endp[1] ));
	}
public:
	int m_endp[2];
	boost::shared_ptr<vector<int> >  m_ptindex; // shared triangles in this edge
};

class TriangleSet{
public:
	template<typename PointT>
	void addtriangle(const TriangleIndex& tri,boost::shared_ptr<pcl::PointCloud<PointT> >& m_pc){
		int index_of_tri=m_vtindex.size();
		m_vtindex.push_back(boost::shared_ptr<TriangleIndex>(new TriangleIndex(tri)));
		m_nv.push_back(boost::shared_ptr<CNormalVector>(new CNormalVector(m_pc->points[tri.m_tindex[0]],\
			m_pc->points[tri.m_tindex[1]],m_pc->points[tri.m_tindex[2]])));

		int v[3];
		tri.getVertexes(v);
		set<TriangleEdge >::iterator it_edge;
		TriangleEdge tmp1(v[0],v[1]);
		it_edge=m_edges.find(tmp1);
		if(it_edge==m_edges.end())
		{
			tmp1.addtriangle(index_of_tri);
			m_edges.insert(tmp1);
		}
		else
		{
			it_edge->addtriangle(index_of_tri);
		}
		TriangleEdge tmp2(v[1],v[2]);
		it_edge=m_edges.find(tmp2);
		if(it_edge==m_edges.end())
		{
			tmp2.addtriangle(index_of_tri);
			m_edges.insert(tmp2);
		}
		else
		{
			it_edge->addtriangle(index_of_tri);
		}
		TriangleEdge tmp3(v[2],v[0]);
		it_edge=m_edges.find(tmp3);
		if(it_edge==m_edges.end())
		{
			tmp3.addtriangle(index_of_tri);
			m_edges.insert(tmp3);
		}
		else
		{
			it_edge->addtriangle(index_of_tri);
		}
	}
	inline size_t size(){return m_vtindex.size();}
public:
	vector<boost::shared_ptr<TriangleIndex> > m_vtindex; // list for all triangles
	vector<boost::shared_ptr<CNormalVector> > m_nv; // normal vectors of each triangle
	vector<boost::shared_ptr<TriangleIndex> > m_vtindex_backup; // filtered triangles
	set<TriangleEdge > m_edges; // list for all edges
};

class CTriangleMesh
{
public:
	CTriangleMesh();
	CTriangleMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& );
	~CTriangleMesh();
public:
	void compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_input,\
		boost::shared_ptr<Triangles>& m_trianlges);
	// Add Point Cloud into Cells
	bool FromPt2Cell();
	void AdjustCrossLinks(int index_pt,IPt index_cell);
	void GetVertex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
	void GetOutlierVertex(pcl::PointCloud<pcl::PointXYZRGB>::Ptr&);
	void GetOutlierTriIndices(vector<pcl::Vertices>& rIndices,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pOut);
	void GetTriIndices(vector<pcl::Vertices>&);
	
	// Generate TriangleMesh depending on Cells
	void FromCell2TriMesh();

	// Merge near triangles into one set (Polygon)
	// Then delete those small enough
	static int m_filter_threshold_for_small_polygon;
	void FilterSmallPolygon();

public:
	// Vertexes 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pVertex;
	// Bounding box index of each point
	map<IPt,int> m_CellIndex;
	vector<boost::shared_ptr<CVtx> > m_CellSet;
	// Triangle Meshes
	boost::shared_ptr<TriangleSet> m_pTriSet;

public:
	template<typename PT>
	inline IPt get_index_point(const PT& sp){
		float x,y,z;
		x=sp.x;		y=sp.y;		z=sp.z;
		if(_isnan(x) || _isnan(y) || _isnan(z))
			return IPt();
		int lx = floor(( x*100 )+0.5);
		lx>>=2;//divide by cell_size
		int ly = floor(( y*100 )+0.5); 
		ly>>=2;//divide by cell_size
		int lz = floor(( z*100 )+0.5); 
		lz>>=2;//divide by cell_size
		return IPt(lx,ly,lz);
	}

private:
	void InitVertexes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& _pPC);
	template<typename PT>
	inline double getdis(const PT& pt1, const PT& pt2){
		return sqrt(_square(pt1.x-pt2.x)+_square(pt1.y-pt2.y)+_square(pt1.z-pt2.z));
	}
	template<typename T>
	inline double _square(T x1){return x1*x1;}
	template<typename PT>
	inline void disloc(const PT& pt,bool _lineswitch=false){
		cout<<"(x,y,z):"<<"("<<pt.x<<","<<pt.y<<","<<pt.z<<")";
		if(_lineswitch)
			cout<<endl;
	}
	template<typename PT>
	inline void discellIndex(const PT& pt, bool _lineswitch=false){
		IPt tmp=get_index_point(pt);
		cout<<"Cell index: (x,y,z) "<<"("<<tmp.ix<<","<<tmp.iy<<","<<tmp.iz<<")";
		if(_lineswitch)
			cout<<endl;
	}
};


#endif