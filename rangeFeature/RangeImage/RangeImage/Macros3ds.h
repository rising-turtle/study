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

// ����3ds��һЩ��ʹ�õ���ChunkID
// ��Chunk,��ÿ���ļ��Ŀ�ʼλ��
const WORD PRIMARY=0x4D4D;
const WORD M3D_VERSION=0x0002;
	const WORD PRIM_EDIT = 0x3D3D;					// ChunkID��3dsģ��
	const WORD MESH_VERN = 0x3D3E;					// ChunkID������汾
		const WORD EDIT_MAT = 0xAFFF;				// ChunkID������
			const WORD MAT_NAME = 0xA000;			// ChunkID����������
			const WORD MAT_AMB  = 0xA010;			// ChunkID�����ʻ��������ԣ�ûʹ�õ���
			const WORD MAT_DIF  = 0xA020;			// ChunkID����������������
			const WORD MAT_SPE  = 0xA030;			// ChunkID�����ʾ��淴�����ԣ�ûʹ�õ���
			const WORD MAT_MAP =  0xA200;			// ChunkID�����ʵ�����
				const WORD MAP_NAME = 0xA300;		// ChunkID�����������

const WORD EDIT_OBJECT = 0x4000;			// ChunkID��3ds������桢�����Ϣ
	const WORD OBJECT_INFO = 0x4100;		// ChunkID���������Ҫ��Ϣ
		const WORD OBJECT_VERTEX = 0x4110;	// ChunkID������Ķ�����Ϣ
		const WORD OBJECT_FACET = 0x4120;	// ChunkID�����������Ϣ
			const WORD FACET_MAT = 0x4130;	// ChunkID��������еĲ���
			const WORD FACET_SMOOTH =0x4150;// ChunkID����⻬��Ϣ��ûʹ�õ���
		const WORD OBJECT_UV = 0x4140;		// ChunkID������������Ϣ
		const WORD OBJECT_LOCAL = 0x4160;

const WORD PRIM_KEY=0xB000;						// ChunkID�����еĹؼ�֡��Ϣ��ûʹ�õ���
const WORD COLOR_BYTE=0x0011;						// ChunkID����ɫ

// �����ַ���
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

// 2ά����
struct MyVector2
{
	float x, y;
};

// 3ά����
struct MyVector3
{
public:
	// ������ʼ��
	MyVector3() {}
	MyVector3(float X, float Y, float Z)	{ x = X; y = Y; z = Z; }
	// �������
	MyVector3 operator+(MyVector3 vVector)	{ return MyVector3(vVector.x + x, vVector.y + y, vVector.z + z); }
	// �������
	MyVector3 operator-(MyVector3 vVector)	{ return MyVector3(x - vVector.x, y - vVector.y, z - vVector.z); }
	// �������
	MyVector3 operator*(float num)		{ return MyVector3(x * num, y * num, z * num); }
	MyVector3 operator/(float num)		{ return MyVector3(x / num, y / num, z / num); }

	float x, y, z;
};

// ����Chunk��Ϣ
typedef struct 
{
	UINT length;									// Chunk�ĳ���
	WORD ID;										// Chunk��ID
} tChunk;

// ��������Ϣ���������������������ֵ��
typedef struct
{
	int vertIndex[3];								// 3�����������ֵ
	int	matID;										// �����Ӧ�Ĳ���ID
} tFace;

// ���������Ϣ��
typedef struct _Material
{
	STRING  matName;								// ���ʵ�����
	STRING  mapName;								// ��������ƣ�bmp��jpg�ȵ��ļ�����
	BYTE	color[3];								// ������ɫ
	//UINT	texureId;								// �����ID��ָ�����������
	bool	isTexMat;								// �ò����ǲ��ǰ���������
	size_t size(){
		size_t ret=0;
		ret+=6*4; // 0xAFFF, 0xA000, 0xA200 0xA300
		ret+=matName.size();
		ret+=mapName.size();
		return ret;
	}
} tMaterial;

// ���浥��3ds����
typedef struct
{
	int  numOfVerts;								// �ö��󶥵�ĸ���
	int  numOfFaces;								// �ö�����ĸ���
	int  numTexVertex;								// �ö�����������ĸ���
	STRING	 objName;								// ������������
	STRING	 matName;								// ������ʵ�����
	MyVector3  *pVerts;								// ���涥������
	MyVector3  *pNormals;								// �����ķ�����
	MyVector2  *pTexVerts;							// ������������
	tFace	 *pFaces;								// ��������Ϣ���������������Ӧ�Ĳ��ʣ�
	size_t size(){
		size_t ret=0;
		ret+=objName.size();						// �������ִ�С
		ret+=7*6; // 0x4000, 0x4100, 0x4110, 0x4140, 0x4160, 0x4120, 0x4130,
		ret=ret + 2 + numOfVerts*sizeof(float)*3;	// ������Ϣ��С
		ret=ret + 2 + numTexVertex*sizeof(float)*2;	// ������Ϣ��С
		ret=ret + 12*sizeof(float);					// ��ת�����С
		ret=ret + 2 + numOfFaces*4*sizeof(WORD);	// ���С
		ret=ret + matName.size();					// ������Ϣ
		ret=ret + 2 + numOfFaces*sizeof(WORD);		// ÿ�����Ӧ�Ĳ�����Ϣ��С
		//ret=ret + sizeof(UINT);					// SMOOTHING ��Ϣ��С
		return ret;
	}
	size_t vertex_size(){
		size_t ret=0;
		ret+= 8;									// chunk + ������
		ret+= sizeof(MyVector3)*numOfVerts;			// �����С
		return ret;
	}
	size_t mat_size(){
		size_t ret=0;
		ret+= 8;									// chunk + ������
		ret+= sizeof(MyVector2)*numOfVerts;			// ���������С
		return ret;
	}
	size_t facet_size(){
		size_t ret=0;
		ret += 8;									// chunk + ������
		ret += 8*numOfFaces;						// ���С
		ret = ret + 6;								// ������Ϣ FACET_MAT 
		ret += matName.size();						// ��������
		ret += 2;									// ���ʶ�Ӧ������
		ret += numOfFaces*2;						// ��ÿ���涼��Ӧ��Ӧ�Ĳ���
		return ret;
	}
	size_t map_size(){
		size_t ret=0;
		ret = ret + 6 + matName.size();				// chunk + ��������
		ret = ret + numOfFaces*2 + 2;				// ÿ���涼��Ӧ������
		return ret;
	}
} t3DObject;

// ��������3dsģ��
typedef struct
{
	int numOfObjects;								// 3ds����ĸ���
	int numOfMaterials;								// 3ds���ʵĸ���
	vector<tMaterial> pMaterials;					// ����3ds����
	vector<t3DObject> pObject;						// ����3ds����
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