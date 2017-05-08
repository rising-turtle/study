#ifndef FROMPC2TRIANGLES
#define FROMPC2TRIANGLES

#include "preheader.h"
#include "globaldefinition.h"

typedef struct _triangle{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud;
	vector<int> m_indices;
	string m_name;
}Triangles;

class CFromPC2Triangle{
public:
	CFromPC2Triangle();
	~CFromPC2Triangle();
	void compute(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_input,boost::shared_ptr<Triangles>& m_trianlges);
	void FilterDensePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sparse, vector<int>& out_indices);
public:
	template<typename PT>
	inline IPt get_index_point(const PT& sp){
		float x,y,z;
		x=sp.x;		y=sp.y;		z=sp.z;
		if(_isnan(x) || _isnan(y) || _isnan(z))
			return IPt();
		int lx = floor(( x*100 )+0.5);
		lx/=CVoxelGridAndNormal::m_voxelleaf;/*>>=2*/;//divide by cell_size
		int ly = floor(( y*100 )+0.5); 
		ly/=CVoxelGridAndNormal::m_voxelleaf;//divide by cell_size
		int lz = floor(( z*100 )+0.5); 
		lz/=CVoxelGridAndNormal::m_voxelleaf;//divide by cell_size
		return IPt(lx,ly,lz);
	}
};


#endif