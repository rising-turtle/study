#ifndef _3DS_H
#define _3DS_H

#include "Macros3ds.h"

// C3DSModel��
class C3DSModel
{
public:
	C3DSModel();
	~C3DSModel();
	BOOL Load(const char *);								// ����3ds�ļ�
	//void Render(void);								// ����3dsģ��
	void Release(void);								// �ͷ�3dsģ����Դ
public:
	void ReadChunk(tChunk *);						// ��ȡ3ds��һ��Chunk��Ϣ��Chunk��ID�����ȣ�

	UINT ReadPrimary(UINT n);						// ��ȡ3ds�ļ���ҪChunk
		UINT ReadEdit(UINT n);						// ��ȡ3ds�������༭Chunk
			UINT ReadObject(UINT n);						// ��ȡ3ds����
				UINT ReadObjectInfo(t3DObject *,UINT n);	// ��ȡ3ds������Ϣ
					UINT ReadFacetInfo(t3DObject *,UINT n);	// ��ȡ����Ϣ
		UINT ReadMaterial(UINT n);					// ��ȡ����
			UINT ReadMatDif(tMaterial *, UINT n);	// ��ȡ���ʵ�����������
			UINT ReadMatMap(tMaterial *, UINT n);	// ��ȡ���ʵ�����
		UINT ReadKeyframe(UINT n);					// ��ȡ֡��Ϣ��δʹ�ã�

	BYTE ReadByte(void);							// ���ļ��ж�ȡ1���ֽ�
	WORD ReadWord(void);							// ���ļ��ж�ȡ2���ֽ�
	UINT ReadUint(void);							// ���ļ��ж�ȡ4���ֽ�
	float ReadFloat(void);							// ���ļ��ж�ȡ������
	UINT ReadString(STRING *);						// ���ļ��ж�ȡ�ַ����������ַ������ȣ�

	MyVector3 Cross(MyVector3, MyVector3);				// �����������Ĳ��
	MyVector3 Normalize(MyVector3);						// ������λ��
	void ComputeNormals(void);						// ���㶥�㷨����

public:
	FILE *m_FilePtr;								// 3ds�ļ�ָ��
	t3DModel m_3DModel;	
	//TextureTga textga;
	// ����3dsģ��
};

#endif