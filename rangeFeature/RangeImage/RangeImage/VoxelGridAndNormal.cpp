#include "VoxelGridAndNormal.h"
#include "HashTable.h"
#include "Hash_Func.h"
#include "windows.h"
#include "pcl/features/normal_3d.h"
#include "QuickSort.h"
#include "boost/dynamic_bitset/dynamic_bitset.hpp"
#include "point_conversion.h"
#include "MyICP.h"
#include "Macros3ds.h"
#include "Writer.h"
#include "stdlib.h"
#include <cmath>

#define COS8 0.99026806874157031508377486734485
#define COS10 0.98480775301220805936674302458952
#define COS18 0.95
#define COS15 0.9659258262890682867497431997289
#define COS5 0.995 //0.99619469809174553229501040247389
#define COS20 0.93969262078590838405410927732473
#define COS30 0.86602540378443864676372317075294
#define COS70 0.34202014332566873304409961468226
int CVoxelGridAndNormal::m_voxelleaf=2; // 2cm*2cm*2cm 
float CVoxelGridAndNormal::m_angle_threshold=COS30;//COS8;
float CVoxelGridAndNormal::m_hue_threshold=5;	// 减小色彩差阀值
float CVoxelGridAndNormal::m_search_radius=0.1;//0.1;	// 增大搜索范围
float CVoxelGridAndNormal::m_planedis_threshold=0.05;//0.02; // plane dis threshold 
float CVoxelGridAndNormal::m_planesize_threshold=1500; // plane size threshold
float CVoxelGridAndNormal::m_floor_level=-10000;
float CVoxelGridAndNormal::m_hue_normal_threshold=1.0; // 最大为10'+delta(10)的偏差

int CVoxelGridAndNormal::m_points_threshold=800; // 面上最少包含的点数

#define MY_GET_FEXT(fname,fext) \
{std::string::size_type d = fname.find_last_of('.'); \
	fext = (d!=std::string::npos) ? fname.substr(d+1) : "";}

#define VERTICAL_NORMAL 0.9
#define HORIZONTAL_NORMAL 0.9
#define DELTA_NORMAL 0.01
#define WALL_SIZE 2000
#define DELTA_FLOOR 0.1

CVoxelGridAndNormal::~CVoxelGridAndNormal(){}

CVoxelGridAndNormal::CVoxelGridAndNormal():
m_pFilterPC(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pOriPC(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pNormals(new pcl::PointCloud<pcl::Normal>),
m_pindices(new vector<int>),
m_tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>),
m_p3dswriter(new C3DWriter)
{}
CVoxelGridAndNormal::CVoxelGridAndNormal(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc):
m_pFilterPC(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pOriPC(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pNormals(new pcl::PointCloud<pcl::Normal>),
m_pindices(new vector<int>),
m_tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>),
m_p3dswriter(new C3DWriter)
{
	m_pOriPC=m_pc;
	FilterDensePC(m_pOriPC,m_pFilterPC,*m_pindices);
	m_tree->setInputCloud(m_pFilterPC);

	CalNormalsWithIndices(m_pFilterPC,m_pindices,m_pNormals);
	//CalNormalsWithIndices(m_pOriPC,m_pindices,m_pNormals);

	// 均值滤波，针对法向量和色彩
	MeanFilter(m_pFilterPC,m_pNormals);
}
CVoxelGridAndNormal::CVoxelGridAndNormal(string file_name):
m_pFilterPC(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pOriPC(new pcl::PointCloud<pcl::PointXYZRGB>),
m_pNormals(new pcl::PointCloud<pcl::Normal>),
m_pindices(new vector<int>),
m_tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>),
m_p3dswriter(new C3DWriter)
{
	Init(file_name);
}

void CVoxelGridAndNormal::Init(string file_name)
{
	if(pcl::io::loadPCDFile(file_name,*m_pOriPC)==-1)
	{
		cout<<"failed to load file: "<<file_name<<endl;
	}
	else{
		FilterDensePC(m_pOriPC,m_pFilterPC,*m_pindices);
		m_tree->setInputCloud(m_pFilterPC);

		CalNormalsWithIndices(m_pFilterPC,m_pindices,m_pNormals);
		//CalNormalsWithIndices(m_pOriPC,m_pindices,m_pNormals);
		
		// 均值滤波，针对法向量和色彩
		MeanFilter(m_pFilterPC,m_pNormals);

	}
}
// 均值滤波，减小噪音的影响，为后续的切割做准备
void CVoxelGridAndNormal::MeanFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc,pcl::PointCloud<pcl::Normal>::Ptr& m_normal)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr tmpNV(new pcl::PointCloud<pcl::Normal>);

	// 准备KD搜索
	int k=9;
	double dis_thre=0.01;
	std::vector<int> nn_indices;
	std::vector<float> nn_distances;
	for(size_t i=0;i<m_pc->points.size();i++)
	{
		pcl::PointXYZRGB& sp = m_pc->points[i];
		pcl::Normal& nv = m_normal->points[i];
		m_tree->nearestKSearch(i,k,nn_indices,nn_distances);
		int back_index=k-1;
		while(nn_distances[back_index]>=dis_thre) back_index--;
		// 距离点i在0.1m之内的点小于3个，则舍弃这个outlier
		if(back_index<2)
		{
			continue;
		}
		// 否则均值化这个点的值
		pcl::PointXYZRGB total_pt=sp;
		pcl::Normal total_nv=nv;
		unsigned int R,G,B;
		R=sp.r; G=sp.g; B=sp.b;
		for(int j=1;j<=back_index;j++)
		{
			pcl::PointXYZRGB& tmpsp = m_pc->points[nn_indices[j]];
			pcl::Normal& tmpnv = m_normal->points[nn_indices[j]];
			total_nv.normal_x+=tmpnv.normal_x; total_nv.normal_y+=tmpnv.normal_y; total_nv.normal_z+=tmpnv.normal_z;
			//total_pt.rgb+=tmpsp.rgb;
			//total_pt.r+=tmpsp.r; total_pt.g+=tmpsp.g; total_pt.b+=tmpsp.b;
			R+=tmpsp.r; G+=tmpsp.g; B+=tmpsp.b;
		}
		total_nv.normal_x/=(back_index+1); total_nv.normal_y/=(back_index+1); total_nv.normal_z/=(back_index+1);
		total_pt.r=R/(back_index+1); total_pt.g=G/(back_index+1); total_pt.b=B/(back_index+1);
	/*	total_pt.r=(unsigned char)total_pt.r/(back_index+1); 
		total_pt.g/=(double)(back_index+1); 
		total_pt.b/=(double)(back_index+1);*/
		// 保存均值化之后的点
		tmpPC->points.push_back(total_pt);
		tmpNV->points.push_back(total_nv);
	}
	// 输出点云与相应的法向量
	m_pc.swap(tmpPC);
	m_normal.swap(tmpNV);
	m_tree->setInputCloud(m_pc);
}

// 计算当前点云，计算矢量化点云
void CVoxelGridAndNormal::computeVector()
{
	// 1, 根据HSV分割点云为若干patches
	SegmentToSurfacePatches(30,CVoxelGridAndNormal::HUE);
	// 2, 计算并获取每个patch的属性：重心+法向量
	CalculateSurfaceAttribute();	
	// 3, 从patch找到相应的面
	FindPlanes(m_plane_index);
	// 4, 合并相近的面
	BlendPlanes(m_plane_index);		
	// 5, 将靠近面的杂点加入面中
	Push2Planes();						 
	// 6, 合并相近的面
	BlendPlanes(m_plane_index);	 
	// 7, 删除很小的面
	DelelteSmallPlane();					
	// 8, 调整面上点的位置
	AdjustPtInPlanes();					
}

// file_in	输入.pcd的点云文件
// file_out 输出.3ds的矢量化文件
void CVoxelGridAndNormal::compute(string file_in, string file_out)
{
	string suffix;
	MY_GET_FEXT(file_in,suffix);
	if(strcmp(suffix.c_str(),"pcd")!=0)
	{
		cout<<"input file should be *.pcd!"<<endl;
		return ;
	}
	MY_GET_FEXT(file_out,suffix);
	if(strcmp(suffix.c_str(),"3ds")!=0)
	{
		cout<<"output file should be *.3ds"<<endl;
		return ;
	}
	vector<boost::shared_ptr<Triangles> > tmpTriangles;
	Init(file_in);
	compute(tmpTriangles);
	Prepare3dsContent(tmpTriangles);
	m_p3dswriter->write(file_out);
}


void CVoxelGridAndNormal::PrepareMat(string mat_name,tMaterial& mat)
{
	stringstream file_name_stream;
	file_name_stream<<mat_name<<".jpg";
	string file_name=file_name_stream.str();
	memset(mat.matName.string,0,128);
	memset(mat.mapName.string,0,128);
	strcpy(mat.matName.string,mat_name.c_str());
	strcpy(mat.mapName.string,file_name.c_str());
}

// 保存场景信息在3ds的中间数据结构中
void CVoxelGridAndNormal::Prepare3dsContent(vector<boost::shared_ptr<Triangles> >& trindex)
{
	srand((unsigned int)(time(NULL)));
	// 新的材质
	tMaterial newMaterial = {0};
	t3DModel& m_3dmodel=m_p3dswriter->m_3DModel;
	newMaterial.isTexMat=true;

	
	// 增加墙的材质
	PrepareMat("wall",newMaterial);
	m_3dmodel.numOfMaterials ++;
	m_3dmodel.pMaterials.push_back(newMaterial);
	// 增加地面的材质
	PrepareMat("floor",newMaterial);
	m_3dmodel.numOfMaterials ++;
	m_3dmodel.pMaterials.push_back(newMaterial);
	// 增加桌子的材质
	PrepareMat("table",newMaterial);
	m_3dmodel.numOfMaterials ++;
	m_3dmodel.pMaterials.push_back(newMaterial);
	// 增加柜子，或者隔板的材质
	PrepareMat("cupboard",newMaterial);
	m_3dmodel.numOfMaterials ++;
	m_3dmodel.pMaterials.push_back(newMaterial);
	// 未知的材质
	PrepareMat("unknown",newMaterial);
	m_3dmodel.numOfMaterials ++;
	m_3dmodel.pMaterials.push_back(newMaterial);
	
	// 每一个物体加进去
	for(int i=0;i<trindex.size();i++)
	{
		Triangles& cur_tri=*(trindex[i]);
		PLANE_TYPE& planetype=m_surfType[i];

		// 新的3ds物体对象
		t3DObject newObject = {0};
		m_3dmodel.numOfObjects ++;
		// 物体名称
		strcpy(newObject.objName.string,cur_tri.m_name.c_str());

		// 写入顶点的信息
		newObject.numOfVerts = cur_tri.m_cloud->points.size();
		newObject.pVerts = new MyVector3[newObject.numOfVerts];
		memset(newObject.pVerts, 0, sizeof(MyVector3) * newObject.numOfVerts);
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr& tmpPC=cur_tri.m_cloud;
		for(int j=0;j<newObject.numOfVerts;j++)
		{
			pcl::PointXYZRGB& sp=tmpPC->points[j];
			newObject.pVerts[j].x=sp.x;
			newObject.pVerts[j].y=sp.y;
			newObject.pVerts[j].z=sp.z;
		}
		// 写入顶点纹理的信息
		newObject.numTexVertex = newObject.numOfVerts;
		newObject.pTexVerts = new MyVector2[newObject.numTexVertex];
		memset(newObject.pTexVerts, 0, sizeof(MyVector2) * newObject.numTexVertex);
		if(planetype==PLANE_TYPE::UNKNOWN)
		{
			for(int k=0;k<newObject.numOfVerts;k++)
			{
				int rand_u=rand()%1000;
				int rand_v=rand()%1000;
				float tmp_u=(float)(rand_u)/1000.0f;
				float tmp_v=(float)(rand_v)/1000.0f;
				newObject.pTexVerts[k].x=tmp_u;
				newObject.pTexVerts[k].y=tmp_v;
			}
		}
		else
		{
			assert(newObject.numTexVertex==4);
			// 面的四个顶点，
			newObject.pTexVerts[0].x=0.1;
			newObject.pTexVerts[0].y=0.1;
			newObject.pTexVerts[1].x=0.9;
			newObject.pTexVerts[1].y=0.1;
			newObject.pTexVerts[2].x=0.1;
			newObject.pTexVerts[2].y=0.9;
			newObject.pTexVerts[3].x=0.9;
			newObject.pTexVerts[3].y=0.9;
		}
		// 写入面的信息
		newObject.numOfFaces = cur_tri.m_indices.size()/3;
		newObject.pFaces = new tFace[newObject.numOfFaces];
		memset(newObject.pFaces, 0, sizeof(tFace) * newObject.numOfFaces);
		vector<int>& indices=cur_tri.m_indices;
		cout<<"indices size: "<<indices.size()<<endl;
		// 读取面索引值(第4个值为3dMAX使用的参数，舍弃)
		for (int j=0; j< newObject.numOfFaces; j++)
		{
			newObject.pFaces[j].vertIndex[0] = indices[j*3];
			newObject.pFaces[j].vertIndex[1] = indices[j*3+1];
			newObject.pFaces[j].vertIndex[2] = indices[j*3+2];
		}

		if(planetype==PLANE_TYPE::WALL)
		{
			for(int j=0;j< newObject.numOfFaces;j++)
				newObject.pFaces[j].matID=0;
			strcpy(newObject.matName.string,"wall");
		}
		else if(planetype==PLANE_TYPE::FLOOR)
		{
			for(int j=0;j< newObject.numOfFaces;j++)
				newObject.pFaces[j].matID=1;
			strcpy(newObject.matName.string,"floor");
		}
		else if(planetype==PLANE_TYPE::TABLE)
		{
			for(int j=0;j<newObject.numOfFaces;j++)
				newObject.pFaces[j].matID=2;
			strcpy(newObject.matName.string,"table");
		}
		else if(planetype==PLANE_TYPE::CUPBOARD)
		{
			for(int j=0;j<newObject.numOfFaces;j++)
				newObject.pFaces[j].matID=3;
			strcpy(newObject.matName.string,"cupboard");
		}
		else if(planetype==PLANE_TYPE::UNKNOWN)
		{
			for(int j=0;j<newObject.numOfFaces;j++)
				newObject.pFaces[j].matID=4;
			strcpy(newObject.matName.string,"unknown");
		}
		// 保存3ds对象
		m_3dmodel.pObject.push_back(newObject);
	}
	cout<<"objects: "<<m_3dmodel.pObject.size()<<endl;
}

void CVoxelGridAndNormal::FilterDensePC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
				   pcl::PointCloud<pcl::PointXYZRGB>::Ptr& sparse, vector<int>& out_indices)
{
	/*map<IPt,int> record_map;
	vector<vector<int> > indices_collector;*/

	// using hash table to filter points
	CHashTableSet<hash_func1,IPt> tmpHash;
	double start_count=::GetTickCount();
	for(size_t i=0;i<ori->points.size();i++)
	{
		pcl::PointXYZRGB& sp=ori->points[i];

		if(_isnan(sp.x)|| _isnan(sp.y) || _isnan(sp.z))
			continue;
		// 删掉天花板的信息
		if(sp.y<-1.2)
			continue;
		IPt cur_index=get_index_point(sp);
		if(!tmpHash.find(cur_index)) // new cell point
		{
			out_indices.push_back(i); // record this index in the original PC
			tmpHash.insert(cur_index);
			sparse->points.push_back(sp);
		}
	}
	double end_count=::GetTickCount();
	cout<<"Time consume: "<<(end_count-start_count)/1000<<"ms"<<endl;
	cout<<"origin PC: "<<ori->points.size()<<endl;
	cout<<"sparse PC: "<<sparse->points.size()<<endl; 

	// using set to filter points
	//set<IPt> index_record;
	//double start_count=::GetTickCount();
	//for(size_t i=0;i<ori->points.size();i++)
	//{
	//	pcl::PointXYZRGB& sp=ori->points[i];

	//	if(_isnan(sp.x)|| _isnan(sp.y) || _isnan(sp.z))
	//		continue;
	//	IPt cur_index=get_index_point(sp);
	//	if(index_record.find(cur_index)==index_record.end()) // new cell point
	//	{
	//		out_indices.push_back(i); // record this index in the original PC
	//		index_record.insert(cur_index);
	//		sparse->points.push_back(sp);
	//	}
	//}
	//double end_count=::GetTickCount();
	//cout<<"Time consume: "<<(end_count-start_count)/1000<<"ms"<<endl;
	//cout<<"origin PC: "<<ori->points.size()<<endl;
	//cout<<"sparse PC: "<<sparse->points.size()<<endl; 
}

// calculate normals of of the sparse PC
void CVoxelGridAndNormal::CalNormalsWithIndices(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
												pcl::IndicesPtr in_indices, pcl::PointCloud<pcl::Normal>::Ptr& out_normals)
{
	// set original PC
	pcl::NormalEstimation<pcl::PointXYZRGB,pcl::Normal> ne;
	ne.setInputCloud(ori);

	// set search method
	//pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree(new pcl::KdTreeFLANN<pcl::PointXYZRGB>);
	ne.setSearchMethod(tree);

	// set search indices 
	//ne.setIndices(in_indices);
	
	// set search radius 4cm 
	//ne.setRadiusSearch(0.1); 
	ne.setKSearch(25);

	ne.compute(*out_normals);
}

void CVoxelGridAndNormal::compute(vector<boost::shared_ptr<Triangles> >& trindex)
{
	// 1 segmented into small surface patches
	SegmentToSurfacePatches(100,CVoxelGridAndNormal::HUE);
	// 2 calculate attribute of each surface patch
	CalculateSurfaceAttribute();
	// 3 plane extraction
	FindPlanes(m_plane_index);
	// 4 blend planes
	BlendPlanes(m_plane_index);
	// 5 plane padding
	AnalysisPlanes(m_surfType);
	DeleteNoisyPatches();
	// 6 find edge points of each plane
	GetPlaneEdgePoints(m_edgepts);
	// 7 triangulation
	SurfaceTriangulation(trindex);

}
void CVoxelGridAndNormal::alternatename(string ori, string& out)
{
	static set<string> name_set;
	static map<string, int> name_index;
	if(name_set.find(ori)==name_set.end())
	{
		out=ori;
		name_set.insert(ori);
		name_index.insert(make_pair(ori,0));
	}
	else
	{
		map<string,int>::iterator it=name_index.find(ori);
		it->second++;
		int index=it->second;
		stringstream tmpss;
		tmpss<<it->first<<index;
		out=tmpss.str();
		name_set.insert(out);
	}
}


void CVoxelGridAndNormal::Triangulation(int surf_id, boost::shared_ptr<Triangles>& tri)
{
	static map<string,int> mesh_name_pool;

	if(m_plane_index[surf_id]) // this is plane
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(m_edgepts[surf_id]->makeShared());
		
		tri->m_cloud=tmp;
		tri->m_indices.resize(6);
		tri->m_indices[0]=0; 	tri->m_indices[1]=1;  tri->m_indices[2]=2; 
		tri->m_indices[3]=1; 	tri->m_indices[4]=2;  tri->m_indices[5]=3; 

		switch(m_surfType[surf_id])
		{
		case PLANE_TYPE::FLOOR:
			alternatename("floor",tri->m_name);
			break;
		case PLANE_TYPE::WALL:
			alternatename("wall",tri->m_name);
			break;
		case PLANE_TYPE::TABLE:
			alternatename("table",tri->m_name);
			break;
		default:
			alternatename("plane",tri->m_name);
			break;
		}
	}
	else
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp(m_edgepts[surf_id]->makeShared());
		CTriangleMesh trimesh;
		trimesh.compute(tmp,tri);
		alternatename("unknown",tri->m_name);
	}
}

void CVoxelGridAndNormal::SurfaceTriangulation(vector<boost::shared_ptr<Triangles> >& trindex)
{
	int index_plane=0;
	for(size_t i=0;i<m_plane_index.size();i++)
	{
		boost::shared_ptr<Triangles> m_tri(new Triangles);
		Triangulation(i,m_tri);
		trindex.push_back(m_tri);
	}
}

void CVoxelGridAndNormal::GetPlaneEdgePoints(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& m_edges)
{
	for(size_t i=0;i<m_plane_index.size();i++)
	{
		if(m_plane_index[i])
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
			switch(m_surfType[i])
			{
			case PLANE_TYPE::FLOOR:
				EagePointsforFloor(i,m_pc);
				break;
			case PLANE_TYPE::WALL:
				EdgePointsforWall(i,m_pc);
				break;
			default:
				CaculateProjectEdgePoints(i,m_pc);
				break;
			}
			m_edges.push_back(m_pc);
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
			vector<int>& cursurf=m_surfaces[i];
			for(size_t i=0;i<cursurf.size();i++)
			{
				m_pc->points.push_back(m_pFilterPC->points[cursurf[i]]);
			}
			m_edges.push_back(m_pc);
		}
	}
}
// 获取每个patch的重心和法向量
void CVoxelGridAndNormal::GetPatchInfo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_centorid,pcl::PointCloud<pcl::Normal>::Ptr m_nv)
{
	for(size_t i=0;i<m_surfInfo.size();i++)
	{
		SurfacePTR& cur_patch=m_surfInfo[i];
		pcl::PointXYZRGB centorid;
		pcl::Normal normal;
		centorid.x=cur_patch->m_centorid.x; centorid.y=cur_patch->m_centorid.y; centorid.z=cur_patch->m_centorid.z;
		centorid.r=golden().r; centorid.g=golden().g; centorid.b=golden().b;
		normal.normal_x=cur_patch->m_nv.nx; normal.normal_y=cur_patch->m_nv.ny; normal.normal_z=cur_patch->m_nv.nz;
		m_centorid->points.push_back(centorid);
		m_nv->points.push_back(normal);
	}
}

// 获取Patch的点云数据
void CVoxelGridAndNormal::GetPatchPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc,int patch_index)
{
	if(patch_index<0 || patch_index>=m_surfaces.size())
	{
		cout<<"patch_index exceed patch size "<<endl;
		return;
	}
	for(size_t j=0;j<m_surfaces[patch_index].size();j++)
	{
		pcl::PointXYZRGB sp = m_pFilterPC->points[m_surfaces[patch_index][j]];
		m_pc->points.push_back(sp);
	}
}

void CVoxelGridAndNormal::GetPlanePoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& plane, \
											 pcl::PointCloud<pcl::PointXYZRGB>::Ptr& non_plane )
{
	int color_index=0;
	COLOR tmpCR(0,0,0);
	for(size_t i=0;i<m_plane_index.size();i++)
	{
		if(m_plane_index[i]) // 加入面的信息
		{
			// 获取相应的点云色彩
			color_index=(++color_index)%g_color_number;
			tmpCR=colorindex(color_index);
			for(size_t j=0;j<m_surfaces[i].size();j++)
			{
				pcl::PointXYZRGB sp = m_pFilterPC->points[m_surfaces[i][j]];
				sp.r=tmpCR.r; sp.g=tmpCR.g; sp.b=tmpCR.b;
				plane->points.push_back(sp);
			}
		}
		else	// 加入非面的信息
		{
			for(size_t j=0;j<m_surfaces[i].size();j++)
			{
				pcl::PointXYZRGB sp = m_pFilterPC->points[m_surfaces[i][j]];
				sp.r=white().r; sp.g=white().g; sp.b=white().b;
				non_plane->points.push_back(sp);
			}
		}
	}
}
void CVoxelGridAndNormal::GetPatchSurfaces(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc)
{
	int color_index=0;
	COLOR tmpCR(0,0,0);
	for(size_t i=0;i<m_surfaces.size();i++)
	{
		// 获取相应的点云色彩
		color_index=(++color_index)%g_color_number;
		tmpCR=colorindex(color_index);

		for(size_t j=0;j<m_surfaces[i].size();j++)
		{
			pcl::PointXYZRGB sp = m_pFilterPC->points[m_surfaces[i][j]];
			sp.r=tmpCR.r; sp.g=tmpCR.g; sp.b=tmpCR.b;
			pc->points.push_back(sp);
		}
	}
}


void CVoxelGridAndNormal::GetPatchSurfaces(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc, PLANE_TYPE type)
{
	assert(m_surfaces.size()==m_surfInfo.size());
	assert(m_surfInfo.size()==m_surfType.size());
	for(size_t i=0;i<m_surfInfo.size();i++)
	{
		if(m_surfType[i]==type){
			for(size_t j=0;j<m_surfaces[i].size();j++)
			{	
				pcl::PointXYZRGB& sp=m_pFilterPC->points[m_surfaces[i][j]];
				pc->points.push_back(m_pFilterPC->points[m_surfaces[i][j]]);
			}
		}
	}
}
void CVoxelGridAndNormal::GetSurfaceInfo(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_pc,pcl::PointCloud<pcl::Normal>::Ptr& m_nv)
{
	pcl::Normal tmpnv;
	pcl::PointXYZRGB tmppc;
	for(size_t i=0;i<m_surfInfo.size();i++)
	{
		SurfacePTR& cursuf=m_surfInfo[i];
		tmpnv.normal_x=cursuf->m_nv.nx;
		tmpnv.normal_y=cursuf->m_nv.ny;
		tmpnv.normal_z=cursuf->m_nv.nz;
		tmppc.x=cursuf->m_centorid.x;
		tmppc.y=cursuf->m_centorid.y;
		tmppc.z=cursuf->m_centorid.z;
		m_pc->points.push_back(tmppc);
		m_nv->points.push_back(tmpnv);
	}
}

// 根据旋转矩阵，改变点云位置
void CVoxelGridAndNormal::TransformPatches(Eigen::Matrix4f& HM)
{
	// 获取所有的patch点
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tmpPC(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr m_norPC(new pcl::PointCloud<pcl::Normal>);
	int index=0;
	for(int i=0;i<m_surfaces.size();i++)
		for(int j=0;j<m_surfaces[i].size();j++)
		{
			m_tmpPC->points.push_back(m_pFilterPC->points[m_surfaces[i][j]]);
			m_norPC->points.push_back(m_pNormals->points[m_surfaces[i][j]]);
			m_surfaces[i][j]=index;
			index++;
		}
		Eigen::Matrix3f rot   = HM.block<3, 3> (0, 0);
		Eigen::Vector3f trans = HM.block<3, 1> (0, 3);

	// 旋转法向量，转换点云 
	pcl::transformPointCloud(*m_tmpPC,*m_pFilterPC,HM);
	m_pNormals->points.resize(m_norPC->points.size());
	for(int i=0;i<m_norPC->points.size();i++){
		pcl::Normal tmpNv;
		tmpNv.getNormalVector3fMap() = rot * m_norPC->points[i].getNormalVector3fMap();
		m_pNormals->points[i]=tmpNv;
	}
	
	// 重新计算法向量和重心
	for(int i=0;i<m_surfInfo.size();i++){
		SurfacePTR& pcur = m_surfInfo[i];
		pcur->Reset();
		for(int j=0;j<m_surfaces[i].size();j++){
			pcur->AddPtAndNormal(m_pFilterPC->points[m_surfaces[i][j]],\
				m_pNormals->points[m_surfaces[i][j]]);
		}
		pcur->Compute();
	}

	// 修改面的法向量和重心
	//for(int i=0;i<m_surfInfo.size();i++){
	//	SurfacePTR& psurf = m_surfInfo[i];
	//	pcl::PointXYZ tmpCentorid;
	//	tmpCentorid.getVector3fMap() = rot * psurf->m_centorid.getVector3fMap() + trans;

	//	// Rotate normals
	//	pcl::Normal tmpNormal,curNormal;
	//	curNormal.normal_x = psurf->m_nv.nx; 
	//	curNormal.normal_y = psurf->m_nv.ny;
	//	curNormal.normal_z = psurf->m_nv.nz;
	//	tmpNormal.getNormalVector3fMap() = rot *curNormal.getNormalVector3fMap();

	//	psurf->m_centorid = tmpCentorid;
	//	psurf->m_nv.nx = tmpNormal.normal_x;
	//	psurf->m_nv.ny = tmpNormal.normal_y;
	//	psurf->m_nv.nz = tmpNormal.normal_z;
	//}
}

#define MAXUP(cur,_max,flag) {if(cur>_max) {_max=cur;flag=true;}}
#define MINDO(cur,_min,flag) {if(cur<_min) {_min=cur;flag=true;}}
#define CONTOUR_THRE 0.02

void CVoxelGridAndNormal::EdgePointsforWall(int surf_id,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_edge)
{
	if(m_surfType[surf_id]!=PLANE_TYPE::WALL)
	{
		cout<<"this plane is not wall"<<endl;
		CaculateProjectEdgePoints(surf_id,m_edge);
		return ;
	}
	SurfacePTR& surfInfo=m_surfInfo[surf_id];
	vector<int>& surface=m_surfaces[surf_id];
	vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >& pedge=m_edge->points;
	 
	float max_x,max_y,max_z;
	float min_x,min_y,min_z;
	pcl::PointXYZRGB left_up_pt,left_down_pt,right_up_pt,right_down_pt;
	bool flag;

	if(fabs(surfInfo->m_nv.nx)>=VERTICAL_NORMAL) // yoz plane
	{
		max_y=-10000; min_y=10000; max_z=-10000; min_z=10000;
		for(size_t i=0;i<surface.size();i++)
		{
			pcl::PointXYZRGB& pt=m_pFilterPC->points[surface[i]];
			MAXUP(pt.y,max_y,flag); MAXUP(pt.z,max_z,flag);
			MINDO(pt.y,min_y,flag); MINDO(pt.z,min_z,flag);
		}
		left_up_pt.y=min_y; left_up_pt.z=max_z; left_up_pt.x=surfInfo->m_centorid.x;
		left_down_pt.y=min_y; left_down_pt.z=min_z; left_down_pt.x=surfInfo->m_centorid.x;
		right_up_pt.y=max_y; right_up_pt.z=max_z; right_up_pt.x=surfInfo->m_centorid.x;
		right_down_pt.y=max_y; right_down_pt.z=min_z; right_down_pt.x=surfInfo->m_centorid.x;
    }
	else if(fabs(surfInfo->m_nv.nz)>=VERTICAL_NORMAL) // xoy plane
	{
		max_x=-10000; min_x=10000; max_y=-10000; min_y=10000;
		for(size_t i=0;i<surface.size();i++)
		{
			pcl::PointXYZRGB& pt=m_pFilterPC->points[surface[i]];
			MAXUP(pt.y,max_y,flag); MAXUP(pt.x,max_x,flag);
			MINDO(pt.y,min_y,flag); MINDO(pt.x,min_x,flag);
		}
		left_up_pt.x=min_x; left_up_pt.y=max_y; left_up_pt.z=surfInfo->m_centorid.z;
		left_down_pt.x=min_x; left_down_pt.y=min_y; left_down_pt.z=surfInfo->m_centorid.z;
		right_up_pt.x=max_x; right_up_pt.y=max_y; right_up_pt.z=surfInfo->m_centorid.z;
		right_down_pt.x=max_x; right_down_pt.y=min_y; right_down_pt.z=surfInfo->m_centorid.z;
	}
	else
	{
		CaculateProjectEdgePoints(surf_id,m_edge);
		return ;
	}
	pedge.clear();

	pedge.push_back(left_up_pt);
	pedge.push_back(left_down_pt);
	pedge.push_back(right_up_pt);
	pedge.push_back(right_down_pt);
}

void CVoxelGridAndNormal::EagePointsforFloor(int surf_id,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_edge)
{
	static float extended_area=3.0;
	if(m_surfType[surf_id]!=PLANE_TYPE::FLOOR)
	{
		cout<<"this plane is not floor!"<<endl;
		CaculateProjectEdgePoints(surf_id,m_edge);
		return ;
	}
	SurfacePTR& surfInfo=m_surfInfo[surf_id];
	float max_x,min_x,max_z,min_z;
	max_x=-10000; min_x=10000; max_z=-10000; min_z=10000;
	pcl::PointXYZRGB left_up_pt,left_down_pt,right_up_pt,right_down_pt;
	vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >& pedge=m_edge->points;
	vector<int>& surface=m_surfaces[surf_id];
	bool flag;
	for(size_t i=0;i<surface.size();i++)
	{
		pcl::PointXYZRGB& pt=m_pFilterPC->points[surface[i]];
		MAXUP(pt.x,max_x,flag); MAXUP(pt.z,max_z,flag);
		MINDO(pt.x,min_x,flag); MINDO(pt.z,min_z,flag);
	}
	max_x+=extended_area; min_x-=extended_area; 
	max_z+=extended_area; min_z-=extended_area;
	left_up_pt.x=min_x; left_up_pt.z=max_z; left_up_pt.y=surfInfo->m_centorid.y;
	left_down_pt.x=min_x; left_down_pt.z=min_z; left_down_pt.y=surfInfo->m_centorid.y;
	right_up_pt.x=max_x; right_up_pt.z=max_z; right_up_pt.y=surfInfo->m_centorid.y;
	right_down_pt.x=max_x; right_down_pt.z=min_z; right_down_pt.y=surfInfo->m_centorid.y;
	
	pedge.clear();

	pedge.push_back(left_up_pt);
	pedge.push_back(left_down_pt);
	pedge.push_back(right_up_pt);
	pedge.push_back(right_down_pt);
}

void CVoxelGridAndNormal::CaculateProjectEdgePoints(int surf_id,pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_edge)
{
	if(surf_id<0 || surf_id>= m_surfInfo.size())
	{
		cout<<"range exceed in CaculateProjectEdgePoints()"<<endl;
		return ;
	}
	float max_u=-10000,min_u=10000,max_v=-10000,min_v=10000;
	SurfacePTR& surfInfo=m_surfInfo[surf_id];

	CPose3D pose;
	pcl::MyICP<pcl::PointXYZRGB,pcl::PointXYZRGB> myicp;
	vector<pcl::TMatchingPair> correspondences; // matching result

	// Get the normal estimate at the current point 
	Eigen::Vector3f nc,v_,u_;
	nc[0]=surfInfo->m_nv.nx; nc[1]=surfInfo->m_nv.ny; nc[2]=surfInfo->m_nv.nz;

	// Get a coordinate system that lies on a plane defined by its normal
	v_ = nc.unitOrthogonal ();
	u_ = nc.cross (v_);

	Eigen::Vector3f coords=surfInfo->m_centorid.getVector3fMap();
	Eigen::Vector3f proj_qp_,tmp_;
	vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >& pedge=m_edge->points;
	// Projecting point onto the surface 
	//double dist = nc.dot(coords);
	//proj_qp_ = coords - dist * nc;

	float u,v,w;
	vector<int>& surface=m_surfaces[surf_id];
	bool flag;
	vector<float> set_u;
	vector<float> set_v;
	for (int i = 0; i < surface.size(); i++) // nearest neighbor with index 0 is the query point R_ itself
	{
		pcl::PointXYZRGB& pt=m_pFilterPC->points[surface[i]];
		// Transforming coordinates
		coords=pt.getVector3fMap();
		w=nc.dot(coords);
		proj_qp_=coords-w*nc;
		//tmp_ = pt.getVector3fMap() - proj_qp_;
		//u = tmp_.dot(u_);
		//v = tmp_.dot(v_);
		u=proj_qp_.dot(u_);
		v=proj_qp_.dot(v_);
		flag=false;
		MAXUP(u,max_u,flag); MAXUP(v,max_v,flag);
		MINDO(u,min_u,flag); MINDO(v,min_v,flag);
		if(flag)
		{
			set_u.push_back(u);
			set_v.push_back(v);
			pedge.push_back(pt);
			correspondences.push_back(pcl::TMatchingPair(0,0,pt.x,pt.y,pt.z,u,v,0));
		}
	}
	myicp.leastSquareErrorRigidTransformation6D(correspondences,pose);
	Eigen::Vector4f left_up,left_down,right_up,right_down;
	Eigen::Vector4f left_up_,left_down_,right_up_,right_down_;
	left_up[0]=min_u; left_up[1]=max_v; left_up[2]=0; left_up[3]=1.0;
	left_down[0]=min_u; left_down[1]=min_v; left_down[2]=0; left_down[3]=1.0;
	right_up[0]=max_u; right_up[1]=max_v; right_up[2]=0;	right_up[3]=1.0;
	right_down[0]=max_u; right_down[1]=min_v; right_down[2]=0; right_down[3]=1.0;
	Eigen::Matrix4f HM;
	pose.getHomogeneousMatrix(HM);
	left_up_=HM*left_up;
	left_down_=HM*left_down;
	right_up_=HM*right_up;
	right_down_=HM*right_down;

	pcl::PointXYZRGB left_up_pt,left_down_pt,right_up_pt,right_down_pt;
	for(size_t i=0;i<3;i++)
		left_up_pt.data[i]=left_up_[i];
	
	for(size_t i=0;i<3;i++)
		left_down_pt.data[i]=left_down_[i];
	for(size_t i=0;i<3;i++)
		right_up_pt.data[i]=right_up_[i];
	for(size_t i=0;i<3;i++)
		right_down_pt.data[i]=right_down_[i];
	
	pedge.clear();

	pedge.push_back(left_up_pt);
	pedge.push_back(left_down_pt);
	pedge.push_back(right_up_pt);
	pedge.push_back(right_down_pt);

}

// obtain edge points according to bounding box
void CVoxelGridAndNormal::CalculateEdgePoints(int surface_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_edge)
{
	if(surface_id<0 || surface_id>=m_surfInfo.size())
	{
		cout<<"range exceed in CalculateContourPoints()!"<<endl;
		return;
	}

	const int k =3;// this means how many points edge point search

	SurfacePTR& surfInfo=m_surfInfo[surface_id];
	float x_r[2]; float y_r[2]; float z_r[2];
	x_r[0]=10000; x_r[1]=-10000;
	y_r[0]=10000; y_r[1]=-10000;
	z_r[0]=10000; z_r[1]=-10000;
	bool flag; 

	vector<int>& surf=m_surfaces[surface_id];
	vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >& pedge=m_edge->points;
	for(size_t i=0;i<surf.size();i++){
		pcl::PointXYZRGB& pt=m_pFilterPC->points[surf[i]];
		flag=false;
		MAXUP(pt.x,x_r[1],flag); MAXUP(pt.y,y_r[1],flag); MAXUP(pt.z,z_r[1],flag);
		MINDO(pt.x,x_r[0],flag); MINDO(pt.y,y_r[0],flag); MINDO(pt.z,z_r[0],flag);
	}
	pcl::PointXYZRGB querypt;
	float fx,fy,fz;
	int r,g,b;

	double dis_thrh=0.2;
	double delta_thrh=0.1;
	while(1){
		for(int x_i=0;x_i<=1;x_i++)
			for(int y_i=0;y_i<=1;y_i++)
				for(int z_i=0;z_i<=1;z_i++)
				{
					querypt.x=x_r[x_i];
					querypt.y=y_r[y_i];
					querypt.z=z_r[z_i];
					vector<int> nn_dices;
					vector<float> nn_dists;
					int ret=m_tree->nearestKSearch(querypt,k,nn_dices,nn_dists);
					if(ret==0){
						continue;
					}
					if(nn_dists[0]>dis_thrh) 
						continue;
					fx=fy=fz=0;
					r=g=b=0;
					for(size_t l=0;l<k;l++){
						fx+=m_pFilterPC->points[nn_dices[l]].x;
						fy+=m_pFilterPC->points[nn_dices[l]].y;
						fz+=m_pFilterPC->points[nn_dices[l]].z;
						r+=m_pFilterPC->points[nn_dices[l]].r;
						g+=m_pFilterPC->points[nn_dices[l]].g;
						b+=m_pFilterPC->points[nn_dices[l]].b;
					}
					querypt.x=fx/(float)k;
					querypt.y=fy/(float)k;
					surfInfo->ComputeZ(querypt);
					//querypt.z=fz/(float)k;
					querypt.r=(unsigned char)(r/k);
					querypt.g=(unsigned char)(g/k);
					querypt.b=(unsigned char)(b/k);
					pedge.push_back(querypt);
				}
		if(pedge.size()<4)
		{
			dis_thrh+=delta_thrh;
			pedge.clear();
		}
		else
			break;
	}
}

// calculate points along edge contour
void CVoxelGridAndNormal::CalculateContourPoints(int surface_id, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& m_edge)
{
	if(surface_id<0 || surface_id>=m_surfInfo.size())
	{
		cout<<"range exceed in CalculateContourPoints()!"<<endl;
		return;
	}
	 SurfacePTR& surfInfo=m_surfInfo[surface_id];
	 float min_x=10000; float max_x=-10000;
	 float min_y=10000; float max_y=-10000;
	 float min_z=10000; float max_z=-10000;
     bool flag; 

	 vector<int>& surf=m_surfaces[surface_id];
	 vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >& pedge=m_edge->points;
	 for(size_t i=0;i<surf.size();i++){
		 pcl::PointXYZRGB& pt=m_pFilterPC->points[surf[i]];
		 flag=false;
		 MAXUP(pt.x,max_x,flag); MAXUP(pt.y,max_y,flag); MAXUP(pt.z,max_z,flag);
		 MINDO(pt.x,min_x,flag); MINDO(pt.y,min_y,flag); MINDO(pt.z,min_z,flag);
		 if(flag)
			 pedge.push_back(pt);
	 }
	 vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB> >::iterator it=pedge.begin();
	 for(;it!=pedge.end();)
	 {
		 pcl::PointXYZRGB& pt=*it;
		 if(fabs(pt.x-max_x)<CONTOUR_THRE || fabs(pt.x-min_x)<CONTOUR_THRE || \
			 fabs(pt.y-max_y)<CONTOUR_THRE || fabs(pt.y-min_y)<CONTOUR_THRE || \
			 fabs(pt.z-max_z)<CONTOUR_THRE || fabs(pt.z-min_z)<CONTOUR_THRE)
		 {
			 it++;
		 }
		 else{
			 it=pedge.erase(it);
		 }
	 }
}

// segment to patches saved in m_surfaces
void CVoxelGridAndNormal::SegmentToSurfacePatches(int surface_size,CVoxelGridAndNormal::SEG segmethod)
{
	if(segmethod==SEG::NORMAL){
		SeededNormalSegmentation(m_pFilterPC,m_tree,m_pNormals,CVoxelGridAndNormal::m_search_radius,\
			CVoxelGridAndNormal::m_angle_threshold,m_surfaces);
	}
	else if(segmethod==SEG::HUE){
		SeededHueSegmentation(m_pFilterPC,m_tree,m_pNormals,CVoxelGridAndNormal::m_search_radius,\
			CVoxelGridAndNormal::m_hue_threshold,m_surfaces);
	}
	else if(segmethod==SEG::HUENORMAL){
		SeededHueNormalSegmentation(m_pFilterPC,m_tree,m_pNormals,CVoxelGridAndNormal::m_search_radius,\
			CVoxelGridAndNormal::m_hue_normal_threshold,m_surfaces);
	}
	else{
		cout<<"such method is not rendered!"<<endl;
		return ;
	}
	for(vector<vector<int> >::iterator it=m_surfaces.begin();it!=m_surfaces.end();)
	{
		if((*it).size()<surface_size)
		{
			it=m_surfaces.erase(it);
		}
		else
			it++;
	}
}
bool CVoxelGridAndNormal::IsAdjacent(pcl::PointXYZRGB& pt1,pcl::PointXYZ& pt2){
	pcl::PointXYZRGB pt;
	pt.x=pt2.x; pt.y=pt2.y; pt.z=pt2.z;
	return IsAdjacent(pt1,pt);
}
bool CVoxelGridAndNormal::IsAdjacent(pcl::PointXYZRGB& pt1, pcl::PointXYZRGB& pt2)
{
	if(_dis(pt1,pt2)<=0.15)
		return true;

	pcl::PointXYZRGB midpt;
	midpt.x=(pt1.x+pt2.x)/2;
	midpt.y=(pt1.y+pt2.y)/2;
	midpt.z=(pt1.z+pt2.z)/2;

	vector<int> nn_dices;
	vector<float> nn_dist;

	int ret = m_tree->radiusSearch (midpt, CVoxelGridAndNormal::m_search_radius, \
		nn_dices, nn_dist, std::numeric_limits<int>::max());
	if(ret == -1)
		PCL_ERROR("[pcl::seededHueSegmentation] radiusSearch returned error code -1");
	// Search for sq_idx
	if (!ret) // not adjacent
	{
		return false;
	}
	
	// let's first set this threshold as 1dm=0.1m
	if(_dis(midpt,pt1)<=0.15 || _dis(midpt,pt2)<=0.15){
		return true;
	}
	return (IsAdjacent(pt1,midpt)&&IsAdjacent(pt2,midpt));
}

void CVoxelGridAndNormal::ShowSimilarPlanes(int p1,int p2)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr patch_pts(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::Normal>::Ptr patch_normal(new pcl::PointCloud<pcl::Normal>);
	assert(p1>=0 && p1<m_surfaces.size());
	assert(p2>=0 && p2<m_surfaces.size());
	for(int i=0;i<m_surfaces[p1].size();i++){
		patch_pts->points.push_back(m_pFilterPC->points[m_surfaces[p1][i]]);
		patch_normal->points.push_back(m_pNormals->points[m_surfaces[p1][i]]);
	}
	for(int i=0;i<m_surfaces[p2].size();i++){
		patch_pts->points.push_back(m_pFilterPC->points[m_surfaces[p2][i]]);
		patch_normal->points.push_back(m_pNormals->points[m_surfaces[p2][i]]);
	}
	DisplayPTAndNormal(patch_pts,patch_normal);
}

bool CVoxelGridAndNormal::IsSimilarPlanes(SurfacePTR& p1, SurfacePTR& p2 )
{
	double angle=p1->m_nv.DotProduct(p2->m_nv);
	if(angle<=COS10)
	{
		return false;
	}

	// 点到面的距离不能太大
	double dis = (p1->point2plane(p2->m_centorid)+p2->point2plane(p1->m_centorid))/2;
	double dis_thre = (CalculateDisAccord2Range(p1->m_centorid.z) + CalculateDisAccord2Range(p2->m_centorid.z))/2;
	if(dis >= dis_thre) 
		return false;

	// 对于地面则不用判断相邻性
	if(fabs(p1->m_nv.ny)>0.88)
	{
		return true;
	}
	// 两个面必须相邻
	pcl::PointXYZRGB pt1,pt2;
	pt1.x=p1->m_centorid.x;
	pt1.y=p1->m_centorid.y;
	pt1.z=p1->m_centorid.z;
	pt2.x=p2->m_centorid.x;
	pt2.y=p2->m_centorid.y;
	pt2.z=p2->m_centorid.z;

	// 重心之间的连线与法向量几乎垂直
	pcl::PointXYZ centorid;
	centorid.x = pt1.x-pt2.x;
	centorid.y = pt1.y-pt2.y;
	centorid.z = pt1.z-pt2.z;
	double angle2 = (fabs(p1->m_nv.DotProduct(centorid)) + fabs(p2->m_nv.DotProduct(centorid)))/2; 
	if(angle2 >= COS70)
		return false;

	// 查看两个平面之间的距离
	//double dis_p = (p1->point2plane(p2->m_centorid) + p2->point2plane(p1->m_centorid))/2;
	//double dis_r = (CalculateDisAccord2Range(p1->m_centorid.z) + CalculateDisAccord2Range(p2->m_centorid.z))/2;
	return IsAdjacent(pt1,pt2);
}

bool CVoxelGridAndNormal::IsNoisyPatch(int plane_id, int patch_id)
{
	assert(plane_id>=0 && plane_id<m_plane_index.size());
	assert(patch_id>=0 && patch_id<m_plane_index.size());
	SurfacePTR& patchsurf=m_surfInfo[patch_id];
	SurfacePTR& planesurf=m_surfInfo[plane_id];

	CNormalVector tmpnv;
	tmpnv.nx=planesurf->m_centorid.x-patchsurf->m_centorid.x;
	tmpnv.ny=planesurf->m_centorid.y-patchsurf->m_centorid.y;
	tmpnv.nz=planesurf->m_centorid.z-patchsurf->m_centorid.z;

	float cross=tmpnv.DotProduct(planesurf->m_nv);
	pcl::PointXYZRGB tmppc1,tmppc2;
	tmppc1.x=planesurf->m_centorid.x; 	tmppc1.y=planesurf->m_centorid.y; 	tmppc1.z=planesurf->m_centorid.z; 
	tmppc2.x=patchsurf->m_centorid.x;   tmppc2.y=patchsurf->m_centorid.y;	tmppc2.z=patchsurf->m_centorid.z;

	if(fabs(cross)>=COS15)
	{
		if(IsAdjacent(tmppc1,tmppc2))
			return true;
	}


	float total_dis=0;
	vector<int>& m_surf=m_surfaces[patch_id]; 
	vector<float> dis;
	for(size_t i=0;i<m_surf.size();i++)
	{
		pcl::PointXYZRGB& pt=m_pFilterPC->points[m_surf[i]];
		float curdis=planesurf->point2plane(pt);
		dis.push_back(curdis);
		total_dis+=curdis;
	}
	int _n=m_surf.size();
	float mean_dis=total_dis/(float)(_n);
	if(mean_dis>0.2)
		return false;
	float delta_total_dis=0;
	for(size_t i=0;i<dis.size();i++)
	{
		float cur_delta=(dis[i]-mean_dis)*(dis[i]-mean_dis);
		delta_total_dis+=cur_delta;
	}
	float variance=delta_total_dis/(int)(_n);
	if(variance<0.0025)
		return true;

	return false;
}

// 删除很小的面
void CVoxelGridAndNormal::DelelteSmallPlane()
{
	vector<bool> is_to_delete;
	is_to_delete.resize(m_surfaces.size(),false);
	for(int i=0;i<m_surfaces.size();i++){
		if((m_surfInfo[i])->m_num<CVoxelGridAndNormal::m_points_threshold)
			is_to_delete[i]=true;
	}
	vector<bool> tmpIndex;
	// delete noisy patches
	vector<vector<int> >::iterator it=m_surfaces.begin();
	vector<SurfacePTR>::iterator itinfo=m_surfInfo.begin();
	//vector<PLANE_TYPE>::iterator ittype=m_surfType.begin();
	for(size_t i=0;it!=m_surfaces.end();)
	{
		if(is_to_delete[i])
		{
			it=m_surfaces.erase(it);
			itinfo=m_surfInfo.erase(itinfo);
			//ittype=m_surfType.erase(ittype);
		}
		else
		{
			it++;
			itinfo++;
			//ittype++;
			if(m_plane_index[i])
				tmpIndex.push_back(true);
			else
				tmpIndex.push_back(false);
		}
		i++;
	}
	m_plane_index.swap(tmpIndex);
	assert(m_plane_index.size()==m_surfInfo.size());
	assert(m_surfInfo.size()==m_surfaces.size());
	//cout<<"before deleting noisy patches: "<<tmpIndex.size()<<" surfaces!"<<endl;
	//cout<<"after deleting noisy patches: "<<m_plane_index.size()<<" surfaces!"<<endl;
}

// delete those noisy surfaces
void CVoxelGridAndNormal::DeleteNoisyPatches()
{
	vector<bool> deleted_plane;
	deleted_plane.resize(m_plane_index.size(),false);

	for(size_t i=0;i<m_plane_index.size();i++)
	{
		if(m_plane_index[i])
			continue;
		SurfacePTR& pcur=m_surfInfo[i];
		if(CVoxelGridAndNormal::m_floor_level!=-10000 && 
			fabs(pcur->m_centorid.y-CVoxelGridAndNormal::m_floor_level)<0.1 &&
			fabs(pcur->m_nv.ny)>fabs(pcur->m_nv.nx) && fabs(pcur->m_nv.ny)>fabs(pcur->m_nv.nz))
		{
			deleted_plane[i]=true;
			continue;
		}
		for(size_t j=0;j<m_plane_index.size();j++)
		{
			if(!m_plane_index[j])
				continue;
			if(IsNoisyPatch(j,i))
				deleted_plane[i]=true;
		}
	}
	
	vector<bool> tmpIndex;
	// delete noisy patches
	vector<vector<int> >::iterator it=m_surfaces.begin();
	vector<SurfacePTR>::iterator itinfo=m_surfInfo.begin();
	vector<PLANE_TYPE>::iterator ittype=m_surfType.begin();
	for(size_t i=0;it!=m_surfaces.end();)
	{
		if(deleted_plane[i])
		{
			it=m_surfaces.erase(it);
			itinfo=m_surfInfo.erase(itinfo);
			ittype=m_surfType.erase(ittype);
		}
		else
		{
			it++;
			itinfo++;
			ittype++;
			if(m_plane_index[i])
				tmpIndex.push_back(true);
			else
				tmpIndex.push_back(false);
		}
		i++;
	}
	m_plane_index.swap(tmpIndex);
	assert(m_plane_index.size()==m_surfInfo.size());
	assert(m_surfInfo.size()==m_surfaces.size());

	cout<<"before deleting noisy patches: "<<tmpIndex.size()<<" surfaces!"<<endl;
	cout<<"after deleting noisy patches: "<<m_plane_index.size()<<" surfaces!"<<endl;
}

// blend those segmented planes
void CVoxelGridAndNormal::BlendPlanes(vector<bool>& m_planeIndex)
{
	vector<bool> deleted_plane;
	deleted_plane.resize(m_planeIndex.size(),false);
	for(size_t i=0;i<m_planeIndex.size();i++)
	{
		if(deleted_plane[i]) continue;
		if(m_planeIndex[i]) 
		{
			SurfacePTR& pcur=m_surfInfo[i];
			for(size_t j=0;j<m_planeIndex.size();j++)
			{
				if(deleted_plane[j])
					continue;
				if(j==i || !m_planeIndex[j])
					continue;
				SurfacePTR& nxt=m_surfInfo[j];
				if(IsSimilarPlanes(pcur,nxt)) // blend those two planes
				{
					//cout<<"plane j="<<j<<" blend with i="<<i<<endl;
					//ShowSimilarPlanes(i,j);
					pcur->Blend(*nxt);
					m_surfaces[i].insert(m_surfaces[i].end(),m_surfaces[j].begin(),m_surfaces[j].end());
					deleted_plane[j]=true;
				}
			}
		}
	}
	vector<bool> tmpIndex;
	// delete planes that been merged
	vector<vector<int> >::iterator it=m_surfaces.begin();
	vector<SurfacePTR>::iterator itinfo=m_surfInfo.begin();
	for(size_t i=0;it!=m_surfaces.end();)
	{
		if(deleted_plane[i])
		{
			it=m_surfaces.erase(it);
			itinfo=m_surfInfo.erase(itinfo);
		}
		else
		{
			it++;
			itinfo++;
			if(m_planeIndex[i])
				tmpIndex.push_back(true);
			else
				tmpIndex.push_back(false);
		}
		i++;
	}
	m_planeIndex.swap(tmpIndex);
	assert(m_planeIndex.size()==m_surfInfo.size());
	assert(m_surfInfo.size()==m_surfaces.size());

	//cout<<"before blending: "<<tmpIndex.size()<<" surfaces!"<<endl;
	//cout<<"after blending: "<<m_planeIndex.size()<<" surfaces!"<<endl;
}

// y = a*x^2 + b*x + c
// a = 0.0024 b = -0.0034 c=0.0052
// 根据面的中心距离sensor的range值，给出合理的偏差参数
double CVoxelGridAndNormal::CalculateDisAccord2Range(float range)
{
	static double a = 0.0024;
	static double b = -0.0034;
	static double c = 0.0052;
	static double k = 4;
	if(range < 0 || range > 10)
		cout<<"this point exceed range_error!"<<endl;
	double ret = a*range*range + b*range + c;
	return k*ret;
}
void CVoxelGridAndNormal::AdjustPtInPlanes()	// 调整面上点的位置
{
	for(int i=0;i<m_surfaces.size();i++){
		if(m_plane_index[i]){
			SurfacePTR& psur=m_surfInfo[i];
			for(int j=0;j<m_surfaces[i].size();j++){
				psur->ComputeZ(m_pFilterPC->points[m_surfaces[i][j]]);
			}
		}

	}
}

// 把非面上的点压倒平面上
void CVoxelGridAndNormal::Push2Planes()
{
	// 找到不在平面上的点
	vector<bool> IsInPlane(m_pFilterPC->points.size(),false);
	for(int i=0;i<m_surfaces.size();i++)
	{
		if(m_plane_index[i])
		{
			for(int j=0;j<m_surfaces[i].size();j++)
				IsInPlane[m_surfaces[i][j]] = true;
		}
	}
	for(int i=0;i<IsInPlane.size();i++)
	{
		if(!IsInPlane[i]){
			double min_dis = 10000;
			int index =-1;
			pcl::PointXYZRGB& pt = m_pFilterPC->points[i];

			for(int k=0;k<m_surfaces.size();k++){
				if(!m_plane_index[k]) continue;
				SurfacePTR& pcur = m_surfInfo[k];
				// 计算点到面的距离
				double dis = pcur->point2plane(pt);
				double r_err = CalculateDisAccord2Range(pcur->m_centorid.z);
				if(dis<=r_err && IsAdjacent(pt,pcur->m_centorid))
				{
					if(dis<=min_dis) {min_dis=dis;index=k;}
				}
			}
			if(index!=-1){
				m_surfaces[index].push_back(i);
				(m_surfInfo[index])->AddPtAndNormal(m_pFilterPC->points[i],m_pNormals->points[i]);
			}
		}
	}
	for(int i=0;i<m_surfInfo.size();i++)
	{
		(m_surfInfo[i])->Compute();
	}
}

// find planes in those patches
void CVoxelGridAndNormal::FindPlanes(vector<bool>& m_planeIndex)
{
	m_planeIndex.resize(m_surfaces.size(),false);
	for(size_t i=0;i<m_surfaces.size();i++)
	{
		SurfacePTR& psurf=m_surfInfo[i];
		// 通过点的个数，即patch的尺度
		if(psurf->m_num>=CVoxelGridAndNormal::m_planesize_threshold)
		{
			m_planeIndex[i]=true;
			continue;
		}

		double delta_dis=0;
		pcl::PointXYZ& m_center=psurf->m_centorid; 
		CNormalVector& m_nv=psurf->m_nv;
		for(size_t j=0;j<psurf->m_num;j++)
		{
			pcl::Normal pnv;
			pcl::PointXYZRGB& m_tmpPT=m_pFilterPC->points[m_surfaces[i][j]];
			pnv.normal_x=m_tmpPT.x-m_center.x;
			pnv.normal_y=m_tmpPT.y-m_center.y;
			pnv.normal_z=m_tmpPT.z-m_center.z;
			double cur_dis=pnv.normal_x*m_nv.nx+pnv.normal_y*m_nv.ny+pnv.normal_z*m_nv.nz;
			delta_dis+=fabs(cur_dis);//*cur_dis;
		}
		//delta_dis=sqrt(delta_dis);
		delta_dis/=psurf->m_num;
		//cout<<"plane "<<i<<": "<<delta_dis<<endl;
		
		// 沿着法向量的偏移小于误差偏移
		if(delta_dis<=CalculateDisAccord2Range(fabs(psurf->m_centorid.z)))
		{
			m_planeIndex[i] = true;
		}
		
		// 沿着法向量的偏移小于阀值
	/*	if(delta_dis<=CVoxelGridAndNormal::m_planedis_threshold)
		{
			m_planeIndex[i]=true;
		}*/
	}
}

// calculate local surface attribution 重心和法向量
void CVoxelGridAndNormal::CalculateSurfaceAttribute()
{
	int index;
	for(size_t i=0;i<m_surfaces.size();i++)
	{
		SurfacePTR pSurface(new CSurfaceInfo);
		for(size_t j=0;j<m_surfaces[i].size();j++)
		{
			index=m_surfaces[i][j];
			pcl::Normal& tmpNV=m_pNormals->points[index];
			pcl::PointXYZRGB& sp=m_pFilterPC->points[index];
	/*		if((tmpNV.normal_x*sp.x+tmpNV.normal_y*sp.y+tmpNV.normal_z*sp.z)>0)
			{
				tmpNV.normal_x*=-1.0;
				tmpNV.normal_y*=-1.0;
				tmpNV.normal_z*=-1.0;
			}*/
			//pSurface->AddPtAndNormal(m_pFilterPC->points[index],m_pNormals->points[index]);
			pSurface->AddPtAndNormal(sp,tmpNV);
		}
		pSurface->Compute();
		m_surfInfo.push_back(pSurface);
	}
}

bool CVoxelGridAndNormal::IsFloor(int index)
{
	if(index < 0 || index > m_surfInfo.size())
	{
		cout<<"exceed range in IsFloor()!"<<endl;
		return false;
	}
	SurfacePTR& querySuf=m_surfInfo[index];
	pcl::PointXYZ& pt=querySuf->m_centorid;
	float max_y=pt.y;
	if(max_y<=0) return false;
	for(size_t i=0;i<m_surfInfo.size();i++)
	{
		if(i==index) continue;
		SurfacePTR& tmpSuf=m_surfInfo[i];
		if(tmpSuf->m_centorid.y-max_y>DELTA_FLOOR)
			return false;
	}
	return true;
}
// whether this index of surface is wall
bool CVoxelGridAndNormal::IsWall(int index)
{
	if(index < 0 || index > m_surfInfo.size())
	{
		cout<<"exceed range in IsWall()!"<<endl;
		return false;
	}
	SurfacePTR& querySuf=m_surfInfo[index];
	if(querySuf->m_num>WALL_SIZE)
		return true;
	pcl::PointXYZ& pt=querySuf->m_centorid;
	CNormalVector& nv=querySuf->m_nv;
	bool bfront=false; 
	bool bback=false; 
	for(size_t i=0; i<m_surfInfo.size();i++)
	{
		if(i==index) continue;
		SurfacePTR& tmpSuf=m_surfInfo[i];
		pcl::PointXYZ& sp=tmpSuf->m_centorid;
		pcl::PointXYZ tmp;
		tmp.x=sp.x-pt.x; 
		tmp.y=sp.y-pt.y;
		tmp.z=sp.z-pt.z;	
		double cur_dis=tmp.x*nv.nx+tmp.y*nv.ny+tmp.z*nv.nz;
		if(cur_dis>0){
			bfront=true;
		}
		if(cur_dis<0){
			bback=true;
		}
	}
	if(bfront==true && bback==true)
		return false;
	return true;
}

void CVoxelGridAndNormal::DisplaySurfaceInfo()
{
	for(size_t i=0;i<m_surfInfo.size();i++)
	{
		cout<<"surface "<<i+1<<", ";
		cout<<"NV: ("<<m_surfInfo[i]->m_nv.nx<<","<<m_surfInfo[i]->m_nv.ny<<","<<m_surfInfo[i]->m_nv.nz<<") "<<endl;
		cout<<"CT: ("<<m_surfInfo[i]->m_centorid.x<<","<<m_surfInfo[i]->m_centorid.y<<","<<m_surfInfo[i]->m_centorid.z<<") ";
		cout<<"Num: ("<<m_surfInfo[i]->m_num<<endl;
	}
}

// AnalysisPlanes according to Normal Vector, Centroid Location,  
void CVoxelGridAndNormal::AnalysisPlanes(vector<PLANE_TYPE>& plane_info)
{
	plane_info.resize(m_surfInfo.size(),PLANE_TYPE::UNKNOWN);
	for(size_t i=0;i<m_surfInfo.size();i++)
	{
		if(!m_plane_index[i]) continue;

		SurfacePTR& pSurface=m_surfInfo[i];
		if(fabs(pSurface->m_nv.nx)>=VERTICAL_NORMAL || fabs(pSurface->m_nv.nz)>=VERTICAL_NORMAL )
		{
			if(IsWall(i))
				plane_info[i]=PLANE_TYPE::WALL;
			else 
				plane_info[i]=PLANE_TYPE::CUPBOARD;
		}
		if(fabs(pSurface->m_nv.ny)>=HORIZONTAL_NORMAL)
		{
			if(IsFloor(i))
			{
				plane_info[i]=PLANE_TYPE::FLOOR;
				CVoxelGridAndNormal::m_floor_level=pSurface->m_centorid.y;
			}
			else
				plane_info[i]=PLANE_TYPE::TABLE;
		}
	}
}

// 把那些不属于任何patch的点单独保存起来
void CVoxelGridAndNormal::ReduceNonPatchPoints()
{
	vector<bool> IsInPatch(m_pFilterPC->points.size(),false);
	for(int i=0;i<m_surfaces.size();i++)
		for(int j=0;j<m_surfaces[i].size();j++){
			IsInPatch[m_surfaces[i][j]]=true;
		}
	vector<int> non_patch;
	for(int i=0;i<IsInPatch.size();i++){
		if(IsInPatch[i]) continue;
		non_patch.push_back(i);
	}
	m_surfaces.push_back(non_patch);
	SurfacePTR tmpSurf(new CSurfaceInfo);
	m_surfInfo.push_back(tmpSurf);
	m_plane_index.push_back(false);
}

// 利用Normal和Hue进行分割
void CVoxelGridAndNormal::SeededHueNormalSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
								 boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB> >& tree,
								 pcl::PointCloud<pcl::Normal>::Ptr& normal,
								 float tolerance,
								 float delta_hue_normal,
								 vector<vector<int> >& surfaces)
{
	if (tree->getInputCloud ()->points.size () != ori->points.size ())
	{
		return;
	}
	// Create a bool vector of processed point indices, and initialize it to false
	std::vector<bool> processed (ori->points.size (), false);

	std::vector<int> nn_indices;
	std::vector<float> nn_distances;

	// Process all points in the indices vector
	for (size_t k = 0; k < ori->points.size (); ++k)
	{
		int i = k;
		if (processed[i])
			continue;

		processed[i] = true;

		std::vector<int> seed_queue;
		int sq_idx = 0;
		seed_queue.push_back (i);

		pcl::PointXYZRGB&  sp=ori->points[i];
		pcl::Normal& nv1=normal->points[i];
		pcl::PointXYZHSV h;
		PointXYZRGBtoXYZHSV(sp, h);

		while (sq_idx < static_cast<int> (seed_queue.size ()))
		{
			int ret = tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances, std::numeric_limits<int>::max());
			if(ret == -1)
				PCL_ERROR("[pcl::seededHueSegmentation] radiusSearch returned error code -1");
			// Search for sq_idx
			if (!ret)
			{
				sq_idx++;
				continue;
			}
			for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
			{
				if (processed[nn_indices[j]])                             // Has this point been processed before ?
					continue;

				pcl::PointXYZRGB  p_l;
				p_l = ori->points[nn_indices[j]];
				pcl::PointXYZHSV h_l;
				PointXYZRGBtoXYZHSV(p_l, h_l);
				pcl::Normal& nv2=normal->points[nn_indices[j]];

				bool valid_flag=false;
				// 如果夹角小于5'，则说明是同一个面上的点
				float delta_angle=fabs(nv1.normal_x*nv2.normal_x+nv1.normal_y*nv2.normal_y+nv1.normal_z*nv2.normal_z);
				double delta_hue=fabs(h_l.h-h.h);
				// 目前先设为1，表示最多差10'并且容忍色彩差6
				static double thresh1=delta_hue_normal;
				double angle_degree=R2D(acos(delta_angle));
				double score1=5.0/angle_degree;
				double score2=3.0/(delta_hue);
				double score=score1+score2;
				if(delta_angle >= COS5) // 法向量一致， 同一平面上的点
				{
					valid_flag=true;
				}
				else if(delta_hue<=2) // 颜色一样， 同一个平面上的点
				{
					valid_flag=true;
				}
				else if(score>=thresh1) // 颜色得分和角度得分满足条件
				{
					valid_flag=true;
					// 加上拣选条件，只把最匹配的点加入，可能造成over-segment，
					for(int k=1;k<nn_indices.size();k++)
					{
						if( k==j || processed[nn_indices[k]])
							continue;
						pcl::Normal& nv2=normal->points[nn_indices[k]];
						double tmpscore;
						int n1=nn_indices[k];
						int n2=nn_indices[j];
						// 计算点pj与其它点之间的得分
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
							tmpscore= score1+score2;
						}

						if(tmpscore > 2*score)
						{
							valid_flag=false; // 这不是最好的匹配
							break;
						}
					}
				}
				if(valid_flag)
				{
					seed_queue.push_back (nn_indices[j]);
					processed[nn_indices[j]] = true;
				}
			}
			sq_idx++;
		}
		if(seed_queue.size()>10)
			surfaces.push_back(seed_queue);
	}
}

void CVoxelGridAndNormal::SeededHueSegmentation (pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
							boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB> >& tree,
							pcl::PointCloud<pcl::Normal>::Ptr& normal,
							float tolerance,
							float delta_hue,
							vector<vector<int> >& surfaces)
{
	if (tree->getInputCloud ()->points.size () != ori->points.size ())
	{
		//PCL_ERROR ("[pcl::seededHueSegmentation] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n", tree->getInputCloud ()->points.size (), cloud.points.size ());
		return;
	}
	// Create a bool vector of processed point indices, and initialize it to false
	std::vector<bool> processed (ori->points.size (), false);

	std::vector<int> nn_indices;
	std::vector<float> nn_distances;

	// Process all points in the indices vector
	for (size_t k = 0; k < ori->points.size (); ++k)
	{
		int i = k;
		if (processed[i])
			continue;

		processed[i] = true;

		std::vector<int> seed_queue;
		int sq_idx = 0;
		seed_queue.push_back (i);

		pcl::PointXYZRGB&  sp=ori->points[i];
		pcl::PointXYZHSV h;
		PointXYZRGBtoXYZHSV(sp, h);

		pcl::Normal& nv1=normal->points[i];

		while (sq_idx < static_cast<int> (seed_queue.size ()))
		{
			int ret = tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances, std::numeric_limits<int>::max());
			if(ret == -1)
				PCL_ERROR("[pcl::seededHueSegmentation] radiusSearch returned error code -1");
			// Search for sq_idx
			if (!ret)
			{
				sq_idx++;
				continue;
			}
			for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
			{
				if (processed[nn_indices[j]])                             // Has this point been processed before ?
					continue;

				pcl::PointXYZRGB  p_l;
				p_l = ori->points[nn_indices[j]];
				pcl::PointXYZHSV h_l;
				PointXYZRGBtoXYZHSV(p_l, h_l);

				pcl::Normal& nv2=normal->points[nn_indices[j]];

				if (fabs(h_l.h - h.h) < delta_hue)
				{
					float angle=fabs(nv1.normal_x*nv2.normal_x+nv1.normal_y*nv2.normal_y+nv1.normal_z*nv2.normal_z);
					if(angle>= CVoxelGridAndNormal::m_angle_threshold)
					{
						seed_queue.push_back (nn_indices[j]);
						processed[nn_indices[j]] = true;
					}
				}
			}

			sq_idx++;
		}
		if(seed_queue.size()>10)
			surfaces.push_back(seed_queue);
	}
}

void CVoxelGridAndNormal::SeededNormalSegmentation ( pcl::PointCloud<pcl::PointXYZRGB>::Ptr& ori,
													boost::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZRGB> >& tree,
													pcl::PointCloud<pcl::Normal>::Ptr& normal,
													float tolerance,
													float delta_normal,
													vector<vector<int> >& surfaces)
{
	if (tree->getInputCloud ()->points.size () != ori->points.size ())
	{
		PCL_ERROR ("[pcl::seededHueSegmentation] Tree built for a different point cloud dataset (%zu) than the input cloud (%zu)!\n", tree->getInputCloud ()->points.size (), ori->points.size ());
		return;
	}
	// Create a bool vector of processed point indices, and initialize it to false
	std::vector<bool> processed (ori->points.size (), false);

	/*boost::dynamic_bitset<> processed;
	processed.resize(ori->points.size(),false);*/

	std::vector<int> nn_indices;
	std::vector<float> nn_distances;

	// Process all points in the indices vector
	for (size_t k = 0; k < ori->points.size (); ++k)
	{
		int i = k;//ori->points[k];
		if (processed[i])
			continue;

		processed[i] = true;

		std::vector<int> seed_queue;
		vector<int> in_indices;
		vector<int> out_indices;
		int sq_idx = 0;

		seed_queue.push_back (i);
		out_indices.push_back(i);

		while (sq_idx < static_cast<int> (seed_queue.size ()))
		{
			int ret = tree->radiusSearch (seed_queue[sq_idx], tolerance, nn_indices, nn_distances, std::numeric_limits<int>::max());
			if(ret == -1)
				PCL_ERROR("[pcl::seededHueSegmentation] radiusSearch returned error code -1");
			// Search for sq_idx
			if (!ret)
			{
				sq_idx++;
				continue;
			}
			
			in_indices.clear();

			for (size_t j = 1; j < nn_indices.size (); ++j)             // nn_indices[0] should be sq_idx
			{
				if (processed[nn_indices[j]])                             // Has this point been processed before ?
					continue;
				in_indices.push_back(nn_indices[j]);
			}
			//pcl::Normal sp=m_pNormals->points[seed_queue[sq_idx]];
			GetInliersNormal(seed_queue[sq_idx],m_pNormals,in_indices,out_indices,CVoxelGridAndNormal::m_angle_threshold,seed_queue,processed);
			sq_idx++;
		}
		if(out_indices.size()>10)
			surfaces.push_back(out_indices);
	}
	// This is purely esthetical, can be removed for speed purposes
	//std::sort (indices_out.indices.begin (), indices_out.indices.end ());
}

void CVoxelGridAndNormal::GetInliersNormal(int center_index,pcl::PointCloud<pcl::Normal>::Ptr& normal,
					  vector<int>& in_indices, vector<int>& out_indices,float angle_threshold,vector<int>& seedqueue,vector<bool>& processed)
{
	vector<float> delta_normal;
	pcl::Normal& center=normal->points[center_index];
	//delta_normal.resize(normal->points.size());
	for(size_t i=0;i<in_indices.size();i++)
	{
		pcl::Normal& sp=normal->points[in_indices[i]];
		float angle=fabs(center.normal_x*sp.normal_x+center.normal_y*sp.normal_y+center.normal_z*sp.normal_z);
		if(angle>=angle_threshold || angle_threshold-angle<0.01)
		{
			delta_normal.push_back(angle);
			out_indices.push_back(in_indices[i]);
			processed[in_indices[i]]=true;
			seedqueue.push_back(in_indices[i]);
		}
	}
	//vector<int> normal_index;
	//QuickSort(delta_normal,normal_index);
	// mean ? rms ?
	
}