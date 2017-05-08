#ifndef MACROS3DS_H
#define MACROS3DS_H

#include <windows.h>
#include <math.h>
#include <vector>
#include <string>
using namespace std;

#define  BYTE unsigned char
#define  WORD unsigned short 
#define  UINT unsigned int

// 定义3ds的一些有使用到的ChunkID
// 根Chunk,在每个文件的开始位置
const WORD PRIMARY=0x4D4D;
const WORD M3D_VERSION=0x0002;
	const WORD PRIM_EDIT = 0x3D3D;					// ChunkID：3ds模型
	const WORD MESH_VERN = 0x3D3E;					// ChunkID：网格版本
		const WORD EDIT_MAT = 0xAFFF;				// ChunkID：材质
			const WORD MAT_NAME = 0xA000;			// ChunkID：材质名称
			const WORD MAT_AMB  = 0xA010;			// ChunkID：材质环境光属性（没使用到）
			const WORD MAT_DIF  = 0xA020;			// ChunkID：材质漫反射属性
			const WORD MAT_SPE  = 0xA030;			// ChunkID：材质镜面反射属性（没使用到）
			const WORD MAT_MAP =  0xA200;			// ChunkID：材质的纹理
				const WORD MAP_NAME = 0xA300;		// ChunkID：纹理的名称

const WORD EDIT_OBJECT = 0x4000;			// ChunkID：3ds对象的面、点等信息
	const WORD OBJECT_INFO = 0x4100;		// ChunkID：对象的主要信息
		const WORD OBJECT_VERTEX = 0x4110;	// ChunkID：物体的顶点信息
		const WORD OBJECT_FACET = 0x4120;	// ChunkID：物体的面信息
			const WORD FACET_MAT = 0x4130;	// ChunkID：物体具有的材质
			const WORD FACET_SMOOTH =0x4150;// ChunkID：面光滑信息（没使用到）
		const WORD OBJECT_UV = 0x4140;		// ChunkID：纹理坐标信息
		const WORD OBJECT_LOCAL = 0x4160;

const WORD PRIM_KEY=0xB000;						// ChunkID：所有的关键帧信息（没使用到）
const WORD COLOR_BYTE=0x0011;						// ChunkID：颜色

// 保存字符串
typedef struct
{
	char string[128];
	size_t size(){
		for(size_t i=0;i<128;i++)
			if(string[i]==0)
				return i+1;
		return 128;
	}
} STRING;

// 2维向量
struct MyVector2
{
	float x, y;
};

// 3维向量
struct MyVector3
{
public:
	// 向量初始化
	MyVector3() {}
	MyVector3(float X, float Y, float Z)	{ x = X; y = Y; z = Z; }
	// 向量相加
	MyVector3 operator+(MyVector3 vVector)	{ return MyVector3(vVector.x + x, vVector.y + y, vVector.z + z); }
	// 向量相加
	MyVector3 operator-(MyVector3 vVector)	{ return MyVector3(x - vVector.x, y - vVector.y, z - vVector.z); }
	// 向量点乘
	MyVector3 operator*(float num)		{ return MyVector3(x * num, y * num, z * num); }
	MyVector3 operator/(float num)		{ return MyVector3(x / num, y / num, z / num); }

	float x, y, z;
};

// 保存Chunk信息
typedef struct 
{
	UINT length;									// Chunk的长度
	WORD ID;										// Chunk的ID
} tChunk;

// 保存面信息：顶点与纹理坐标的索引值。
typedef struct
{
	int vertIndex[3];								// 3个顶点的索引值
	int	matID;										// 该面对应的材质ID
} tFace;

// 保存材质信息。
typedef struct _Material
{
	STRING  matName;								// 材质的名称
	STRING  mapName;								// 纹理的名称（bmp，jpg等的文件名）
	BYTE	color[3];								// 材质颜色
	//UINT	texureId;								// 纹理的ID（指向载入的纹理）
	bool	isTexMat;								// 该材质是不是包含有纹理
	size_t size(){
		size_t ret=0;
		ret+=6*4; // 0xAFFF, 0xA000, 0xA200 0xA300
		ret+=matName.size();
		ret+=mapName.size();
		return ret;
	}
} tMaterial;

// 保存单个3ds对象
typedef struct
{
	int  numOfVerts;								// 该对象顶点的个数
	int  numOfFaces;								// 该对象面的个数
	int  numTexVertex;								// 该对象纹理坐标的个数
	STRING	 objName;								// 保存对象的名称
	STRING	 matName;								// 保存材质的名称
	MyVector3  *pVerts;								// 保存顶点坐标
	MyVector3  *pNormals;								// 保存点的法线量
	MyVector2  *pTexVerts;							// 保存纹理坐标
	tFace	 *pFaces;								// 保存面信息（顶点索引及面对应的材质）
	size_t size(){
		size_t ret=0;
		ret+=objName.size();						// 网格名字大小
		ret+=7*6; // 0x4000, 0x4100, 0x4110, 0x4140, 0x4160, 0x4120, 0x4130,
		ret=ret + 2 + numOfVerts*sizeof(float)*3;	// 顶点信息大小
		ret=ret + 2 + numTexVertex*sizeof(float)*2;	// 纹理信息大小
		ret=ret + 12*sizeof(float);					// 旋转矩阵大小
		ret=ret + 2 + numOfFaces*4*sizeof(WORD);	// 面大小
		ret=ret + matName.size();					// 材质信息
		ret=ret + 2 + numOfFaces*sizeof(WORD);		// 每个面对应的材质信息大小
		//ret=ret + sizeof(UINT);					// SMOOTHING 信息大小
		return ret;
	}
	size_t vertex_size(){
		size_t ret=0;
		ret+= 8;									// chunk + 顶点数
		ret+= sizeof(MyVector3)*numOfVerts;			// 顶点大小
		return ret;
	}
	size_t mat_size(){
		size_t ret=0;
		ret+= 8;									// chunk + 顶点数
		ret+= sizeof(MyVector2)*numOfVerts;			// 顶点纹理大小
		return ret;
	}
	size_t facet_size(){
		size_t ret=0;
		ret += 8;									// chunk + 顶点数
		ret += 8*numOfFaces;						// 面大小
		ret = ret + 6;								// 材质信息 FACET_MAT 
		ret += matName.size();						// 材质名称
		ret += 2;									// 材质对应的面数
		ret += numOfFaces*2;						// 把每个面都对应相应的材质
		return ret;
	}
	size_t map_size(){
		size_t ret=0;
		ret = ret + 6 + matName.size();				// chunk + 材质名称
		ret = ret + numOfFaces*2 + 2;				// 每个面都对应该纹理
		return ret;
	}
} t3DObject;

// 保存整个3ds模型
typedef struct
{
	int numOfObjects;								// 3ds对象的个数
	int numOfMaterials;								// 3ds材质的个数
	vector<tMaterial> pMaterials;					// 保存3ds材质
	vector<t3DObject> pObject;						// 保存3ds对象
	size_t size(){
		size_t ret=0;
		for(size_t i=0;i<pObject.size();i++){
			ret+=pObject[i].size();
			//tFace* cursurf=pObject[i].pFaces;
			//ret+=pMaterials[cursurf->matID].matName.size();
		}
		for(size_t i=1;i<pMaterials.size();i++){
			ret+=pMaterials[i].size();
		}
		ret=ret + 6*4 + 8; // PRIMARY, M3D_VER 10, PRIM_EDIT, MESH_VER 10
		return ret;
	}
} t3DModel;

#endif