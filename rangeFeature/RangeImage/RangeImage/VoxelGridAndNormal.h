#ifndef VOXELGRIDANDNORMAL
#define VOXELGRIDANDNORMAL
#include "preheader.h"
#include "globaldefinition.h"
#include "TriangleMesh.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "SurfaceInfo.h"

class C3DWriter;
struct _Material;

#define _pi 3.1415926535897932384626433832795
#define R2D(r) ((r*180.0)/_pi)

class CVoxelGridAndNormal{
public:
	typedef boost::shared_ptr<CSurfaceInfo> SurfacePTR;

	typedef enum _SEG{HUE,NORMAL,HUENORMAL} SEG; 

public:
	CVoxelGridAndNormal();
	CVoxelGridAndNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc);
	CVoxelGridAndNormal(string file_name);
	~CVoxelGridAndNormal();
	void Init(string file_name);
public:
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pFilterPC;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pOriPC;
	pcl::PointCloud<pcl::Normal>::Ptr m_pNormals;
	pcl::IndicesPtr m_pindices;
	boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB> > m_tree;
	vector< vector<int> > m_surfaces; // depend on filtered point cloud
	vector<SurfacePTR> m_surfInfo;
	vector<PLANE_TYPE> m_surfType;
	vector<bool> m_plane_index;
	vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> m_edgepts;

public:
	void compute(vector<boost::shared_ptr<Triangles> >& trindex);
	
	// 计算当前点云，计算矢量化点云
	void computeVector();
public:
	// obtain sparse PC saved in out_indices
	void FilterDensePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sparse,vector<int>& out_indices);
	// calculate normals of of the sparse PC
	void CalNormalsWithIndices(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
		pcl::IndicesPtr in_indices, pcl::PointCloud<pcl::Normal>::Ptr& out_normals);

	// 均值滤波，减小噪音的影响，为后续的切割做准备
	void MeanFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc,\
		pcl::PointCloud<pcl::Normal>::Ptr& m_normal);

	// 根据面的中心距离sensor的range值，给出合理的偏差参数
	double CalculateDisAccord2Range(float range);

	// 把非面上的点压倒平面上
	void Push2Planes();
	void AdjustPtInPlanes();	// 调整面上点的位置

	// 显示相似的平面
	void ShowSimilarPlanes(int p1,int p2);

	// segment to patches saved in m_surfaces
	void SegmentToSurfacePatches(int surface_size,SEG segmethod=SEG::NORMAL);
	void GetPatchSurfaces(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc);					// 获取所有patch的点云信息
	void GetPatchSurfaces(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc, PLANE_TYPE type); // 获取指定类型面的点云信息
	void GetPatchInfo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_centorid,				// 获取每个patch的属性信息
		pcl::PointCloud<pcl::Normal>::Ptr m_nv); 
	void GetPlanePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane,				    // 获取面与非面的点云信息,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& non_plane );
	void GetPatchPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc,int patch_index);  // 获取Patch的点云数据

	// 根据旋转矩阵，改变点云位置
	void TransformPatches(Eigen::Matrix4f& HM);

	// calculate local surface attribution 
	void CalculateSurfaceAttribute();
	// find planes in those patches
	void FindPlanes(vector<bool>& m_planeIndex);
	void BlendPlanes(vector<bool>& m_planeIndex);
	bool IsSimilarPlanes(SurfacePTR& p1, SurfacePTR& p2 );
	bool IsAdjacent(pcl::PointXYZRGB& pt1, pcl::PointXYZRGB& pt2);
	bool IsAdjacent(pcl::PointXYZRGB& pt1,pcl::PointXYZ& pt2);

	// delete those noisy surfaces
	void DeleteNoisyPatches();
	bool IsNoisyPatch(int plane_id, int patch_id);
	void DelelteSmallPlane();		// 删除很小的面

	// 把那些不属于任何patch的点单独保存起来
	void ReduceNonPatchPoints();

	// find points along edge contour
	void CalculateContourPoints(int surface_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_edge);
	void CalculateEdgePoints(int surface_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_edge);
	void CaculateProjectEdgePoints(int surf_id,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_edge);
	void EdgePointsforWall(int surf_id,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_edge);
	void EagePointsforFloor(int surf_id,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_edge);
	void GetPlaneEdgePoints(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& m_edges);

	// triangulation
	void SurfaceTriangulation(vector<boost::shared_ptr<Triangles> >& trindex);
	void Triangulation(int surf_id, boost::shared_ptr<Triangles>& tri);
	
	// extract walls
	void AnalysisPlanes(vector<PLANE_TYPE>& plane_info);
	void DisplaySurfaceInfo();
	void GetSurfaceInfo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc,pcl::PointCloud<pcl::Normal>::Ptr& m_nv);

	bool IsWall(int index);
	bool IsFloor(int index);

	// segment according to hue
	void SeededHueSegmentation (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
		boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB> >& tree,
		pcl::PointCloud<pcl::Normal>::Ptr& normal,
		float tolerance,
		float delta_hue,
		vector<vector<int> >& surfaces);
	// segment according to seed normal
	void SeededNormalSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
		boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB> >& tree,
		pcl::PointCloud<pcl::Normal>::Ptr& normal,
		float tolerance,
		float delta_normal,
		vector<vector<int> >& surfaces);
	void GetInliersNormal(int center_index,pcl::PointCloud<pcl::Normal>::Ptr& normal,
		vector<int>& in_indices, vector<int>& out_indices,float angle_threshold,vector<int>&,vector<bool>& );
	// 利用Normal和Hue进行分割
	void SeededHueNormalSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
		boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB> >& tree,
		pcl::PointCloud<pcl::Normal>::Ptr& normal,
		float tolerance,
		float delta_hue_normal,
		vector<vector<int> >& surfaces);
	
public:
	boost::shared_ptr<C3DWriter> m_p3dswriter;
	void Prepare3dsContent(vector<boost::shared_ptr<Triangles> >& trindex);
	void PrepareMat(string mat_name,struct _Material& mat);
	void compute(string file_in, string file_out);
	void alternatename(string ori, string& out);
public:
	static int m_voxelleaf;
	static float m_angle_threshold;
	static float m_hue_threshold;
	static float m_hue_normal_threshold;
	static float m_search_radius;
	static float m_planedis_threshold;
	static float m_planesize_threshold;
	static float m_floor_level;
	static int   m_points_threshold;
	
	// 计算两个点相似度的得分
	/*inline double get_score(int n1,int n2)
	{
		pcl::Normal& nv1=m_pNormals->points[n1];
		pcl::Normal& nv2=m_pNormals->points[n2];
		pcl::PointXYZRGB np1,np2;
		pcl::PointXYZHSV nps1,nps2;
		np1 = m_pFilterPC->points[n1];
		np2 = m_pFilterPC->points[n2];
		PointXYZRGBtoXYZHSV(np1, nps1);
		PointXYZRGBtoXYZHSV(np2, nps2);
		float delta_angle=fabs(nv1.normal_x*nv2.normal_x+nv1.normal_y*nv2.normal_y+nv1.normal_z*nv2.normal_z);
		double delta_hue=fabs(nps1.h-nps2.h);
		double angle_degree=R2D(acos(delta_angle));
		double score1=5.0/angle_degree;
		double score2=3.0/(delta_hue);
		return (score1+score2);
	}*/

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
	template<typename PT>
	inline double _dis(PT pt1, PT pt2){
		return sqrt((double)(_square(pt1.x-pt2.x)+_square(pt1.y-pt2.y)+_square(pt1.z-pt2.z)));
	}
	template<typename T>
	T _square(T x){return (x*x);}
};

#endif