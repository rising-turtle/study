#ifndef _3DS_H
#define _3DS_H

#include "Macros3ds.h"

// C3DSModel类
class C3DSModel
{
public:
	C3DSModel();
	~C3DSModel();
	BOOL Load(const char *);								// 载入3ds文件
	//void Render(void);								// 绘制3ds模型
	void Release(void);								// 释放3ds模型资源
public:
	void ReadChunk(tChunk *);						// 读取3ds的一个Chunk信息（Chunk的ID及长度）

	UINT ReadPrimary(UINT n);						// 读取3ds文件主要Chunk
		UINT ReadEdit(UINT n);						// 读取3ds物体主编辑Chunk
			UINT ReadObject(UINT n);						// 读取3ds对象
				UINT ReadObjectInfo(t3DObject *,UINT n);	// 读取3ds对象信息
					UINT ReadFacetInfo(t3DObject *,UINT n);	// 读取面信息
		UINT ReadMaterial(UINT n);					// 读取材质
			UINT ReadMatDif(tMaterial *, UINT n);	// 读取材质的漫反射属性
			UINT ReadMatMap(tMaterial *, UINT n);	// 读取材质的纹理
		UINT ReadKeyframe(UINT n);					// 读取帧信息（未使用）

	BYTE ReadByte(void);							// 从文件中读取1个字节
	WORD ReadWord(void);							// 从文件中读取2个字节
	UINT ReadUint(void);							// 从文件中读取4个字节
	float ReadFloat(void);							// 从文件中读取浮点数
	UINT ReadString(STRING *);						// 从文件中读取字符串（返回字符串长度）

	MyVector3 Cross(MyVector3, MyVector3);				// 计算两向量的叉积
	MyVector3 Normalize(MyVector3);						// 向量单位化
	void ComputeNormals(void);						// 计算顶点法线量

public:
	FILE *m_FilePtr;								// 3ds文件指针
	t3DModel m_3DModel;	
	//TextureTga textga;
	// 保存3ds模型
};

#endif