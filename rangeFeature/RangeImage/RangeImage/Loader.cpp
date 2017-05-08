// Loader.cpp: implementation of the Loader class.
//
//////////////////////////////////////////////////////////////////////

#include "Loader.h"
//#include "Texture.h"

// ���캯��
C3DSModel::C3DSModel()
{
	// ��ʼ���ļ�ָ��
	m_FilePtr = NULL;

	// ����һ��Ĭ�ϵĲ��ʣ���ɫ��
	tMaterial defaultMat;
	defaultMat.isTexMat = false;
	strcpy(defaultMat.matName.string, "5DG_Default");
	defaultMat.color[0] = 192;
	defaultMat.color[1] = 192;
	defaultMat.color[2] = 192;
	m_3DModel.pMaterials.push_back(defaultMat);

	// ��ʼ������3DSģ�͵Ľṹ��
	m_3DModel.numOfMaterials = 1;
	m_3DModel.numOfObjects = 0;
}

// ��������
C3DSModel::~C3DSModel()
{
	m_3DModel.pMaterials.clear();
	m_3DModel.pObject.clear();
}

// ����3ds�ļ�
BOOL C3DSModel::Load(const char *strFileName)
{
	char strMessage[128] = {0};
	tChunk chunk = {0};

	// ���ļ�
	m_FilePtr = fopen(strFileName,"rb");

	// ����ļ���ʧ��
	if (!m_FilePtr)
	{
		sprintf(strMessage, "3DS�ļ� %s �����ڣ�", strFileName);
		//MessageBox(NULL, strMessage, "Error", MB_OK);
		return false;
	}

	// ��ȡ3ds�ļ��ĵ�һ��Chunk
	ReadChunk(&chunk);

	// ����Ƿ���3ds�ļ�
	if (chunk.ID != PRIMARY)
	{
		sprintf(strMessage, "��ȡ�ļ� %s ʧ�ܣ�", strFileName);
		//MessageBox(NULL, strMessage, "Error", MB_OK);
		fclose(m_FilePtr);
		return false;
	}

	// ��ʼ��ȡ3ds�ļ�
	ReadPrimary(chunk.length-6);

	// ����ÿ������ķ�����
	//ComputeNormals();

	// �رմ򿪵��ļ�
	fclose(m_FilePtr);
	m_FilePtr = NULL;

	// ��������Ĳ������������
	//for (int i=0; i<m_3DModel.numOfMaterials; i++)
	//{
	//	if (m_3DModel.pMaterials[i].isTexMat)
	//	{
	//		if (!BuildTexture(m_3DModel.pMaterials[i].mapName.string, m_3DModel.pMaterials[i].texureId))
	//		{
	//			// ��������ʧ��
	//			if(!BuildTexture(m_3DModel.pMaterials[i].mapName.string, &textga))
	//			{
	//			sprintf(strMessage, "3DS�����ļ�����ʧ��: %s ��", m_3DModel.pMaterials[i].mapName.string);
	//			MessageBox(NULL, strMessage, "Error", MB_OK);
	//			}
	//		}
	//	}
	//}

	return true;
}

// ���ļ��ж�ȡ1���ֽ�
BYTE C3DSModel::ReadByte(void)
{
	BYTE result = 0;
	fread(&result, 1, 1, m_FilePtr);
	return result;
}

// ���ļ��ж�ȡ2���ֽ�
WORD C3DSModel::ReadWord(void)
{
	return ReadByte() + (ReadByte()<<8);
}

// ���ļ��ж�ȡ4���ֽ�
UINT C3DSModel::ReadUint(void)
{
	return ReadWord() + (ReadWord()<<16);
}

// ���ļ��ж�ȡ������
float C3DSModel::ReadFloat(void)
{
	float result;
	fread(&result, sizeof(float), 1, m_FilePtr);
	return result;
}

// ���ļ��ж�ȡ�ַ����������ַ������ȣ�
UINT C3DSModel::ReadString(STRING *pStr)
{
	int n=0;
	while ((pStr->string[n++]=ReadByte()) != 0)
		;
	return n;
}

// ��ȡ3ds��һ��Chunk��Ϣ
void C3DSModel::ReadChunk(tChunk *pChunk)
{
	fread(&pChunk->ID, 1, 2, m_FilePtr);
	fread(&pChunk->length, 1, 4, m_FilePtr);
}

// ��ȡ3ds�ļ���ҪChunk
UINT C3DSModel::ReadPrimary(UINT n)
{
	UINT count = 0;				// ��Chunk�����Ѷ�ȡ���ֽڼ���
	tChunk chunk = {0};			// ���Ա�����Chunk������
	while (count < n)
	{
		ReadChunk(&chunk);
		switch (chunk.ID)
		{
		case PRIM_EDIT:
			ReadEdit(chunk.length-6);
			break;
		//case PRIM_KEY:
		//	ReadKeyframe(chunk.length-6);
		//	break;
		default:
			fseek(m_FilePtr, chunk.length-6, SEEK_CUR);
			break;
		}
		count += chunk.length;
	}
	return count;
}

// ��ȡ3ds�������༭Chunk
UINT C3DSModel::ReadEdit(UINT n)
{
	UINT count = 0;
	tChunk chunk = {0};
	while(count < n)
	{
		ReadChunk(&chunk);
		switch(chunk.ID)
		{
		case EDIT_MAT:
			ReadMaterial(chunk.length-6);
			break;
		case EDIT_OBJECT:
			ReadObject(chunk.length-6);
			break;
		default:
			fseek(m_FilePtr, chunk.length-6, SEEK_CUR);
			break;
		}
		count += chunk.length;
	}
	return count;
}

// ��ȡ3ds����
UINT C3DSModel::ReadObject(UINT n)
{
	UINT count = 0;
	tChunk chunk = {0};
	// �µ�3ds����
	t3DObject newObject = {0};
	count += ReadString(&newObject.objName);
	m_3DModel.numOfObjects ++;

	while (count < n)
	{
		ReadChunk(&chunk);
		switch (chunk.ID)
		{
		case OBJECT_INFO:
			ReadObjectInfo(&newObject, n-count -6);
			break;
		default:
			fseek(m_FilePtr, chunk.length-6, SEEK_CUR);
			break;
		}
		count += chunk.length;
	}
	// ����3ds����
	m_3DModel.pObject.push_back(newObject);
	return count;
}

// ��ȡ3ds������Ϣ
UINT C3DSModel::ReadObjectInfo(t3DObject *pObj, UINT n)
{
	UINT count = 0;
	tChunk chunk = {0};
    int i;
	while (count < n)
	{
		ReadChunk(&chunk);
		switch (chunk.ID)
		{
		case OBJECT_VERTEX:
			pObj->numOfVerts = ReadWord();
			pObj->pVerts = new MyVector3[pObj->numOfVerts];
			memset(pObj->pVerts, 0, sizeof(MyVector3) * pObj->numOfVerts);
			// �����ȡ��������ֵ
			fread(pObj->pVerts, 1, chunk.length - 8, m_FilePtr);
			// ����y��z����ֵ(����3dMAX����ϵ������OpenGL��ͬ)
			float fTempY;
			for (i = 0; i < pObj->numOfVerts; i++)
			{
				fTempY = pObj->pVerts[i].y;
				pObj->pVerts[i].y = pObj->pVerts[i].z;
				pObj->pVerts[i].z = -fTempY;
			}
			break;
		case OBJECT_FACET:
			ReadFacetInfo(pObj,chunk.length-6);
			break;
		case OBJECT_UV:
			pObj->numTexVertex = ReadWord();
			pObj->pTexVerts = new MyVector2[pObj->numTexVertex];
			memset(pObj->pTexVerts, 0, sizeof(MyVector2) * pObj->numTexVertex);
			// �����ȡ��������ֵ
			fread(pObj->pTexVerts, 1, chunk.length - 8, m_FilePtr);
		/*	for(int i=0;i<pObj->numTexVertex;i++)
			{
				MyVector2& pcur=pObj->pTexVerts[i];
				printf("(%f_u,%f_v),",pcur.x,pcur.y);
			}*/
			break;
		default:
			fseek(m_FilePtr, chunk.length-6, SEEK_CUR);
			break;
		}
		count += chunk.length;
	}
	return count;
}

// ��ȡ����Ϣ
UINT C3DSModel::ReadFacetInfo(t3DObject *pObj, UINT n)
{
	UINT count = 0;
	tChunk chunk = {0};
	pObj->numOfFaces = ReadWord();
	pObj->pFaces = new tFace[pObj->numOfFaces];
	memset(pObj->pFaces, 0, sizeof(tFace) * pObj->numOfFaces);
	// ��ȡ������ֵ(��4��ֵΪ3dMAXʹ�õĲ���������)
	for (int i=0; i<pObj->numOfFaces; i++)
	{
		pObj->pFaces[i].vertIndex[0] = ReadWord();
		pObj->pFaces[i].vertIndex[1] = ReadWord();
		pObj->pFaces[i].vertIndex[2] = ReadWord();
		ReadWord();
	}
	count +=2+pObj->numOfFaces*8;

	STRING matName;
	int t,i;
	int matID = 0;
	while (count < n)
	{
		ReadChunk(&chunk);
		switch (chunk.ID)
		{
		case FACET_MAT:
			ReadString(&matName);			// ��������
			strcpy(pObj->matName.string,matName.string);	//������ʵ�����
			t=ReadWord();					// ���ʶ�Ӧ�������
			// ���Ҷ�Ӧ�Ĳ���
			for ( i=1;i<=m_3DModel.numOfMaterials;i++)
			{
				if (strcmp(matName.string, m_3DModel.pMaterials[i].matName.string) == 0)
				{
					matID = i;
					break;
				}
			}
			// ������������ÿ����󶨲���ID
			while (t>0)
			{
				pObj->pFaces[ReadWord()].matID = matID;
				t--;
			}
			break;
		default:
			fseek(m_FilePtr, chunk.length-6, SEEK_CUR);
			break;
		}
		count += chunk.length;
	}
	return count;
}

// ��ȡ����
UINT C3DSModel::ReadMaterial(UINT n)
{
	UINT count = 0;
	tChunk chunk = {0};
	// �µĲ���
	tMaterial newMaterial = {0};
	m_3DModel.numOfMaterials ++;
	while (count < n)
	{
		ReadChunk(&chunk);
		switch (chunk.ID)
		{
		case MAT_NAME:
			ReadString(&newMaterial.matName);
			break;
		case MAT_DIF:
			ReadMatDif (&newMaterial, chunk.length-6);
			break;
		case MAT_MAP:
			ReadMatMap(&newMaterial, chunk.length-6);
			break;
		default:
			fseek(m_FilePtr, chunk.length-6, SEEK_CUR);
			break;
		}
		count += chunk.length;
	}
	// �����µĲ���
	m_3DModel.pMaterials.push_back(newMaterial);
	return count;
}

// ��ȡ���ʵ�����������
UINT C3DSModel::ReadMatDif (tMaterial *pMat, UINT n)
{
	UINT count = 0;
	tChunk chunk = {0};
	while (count<n)
	{
		ReadChunk(&chunk);
		switch (chunk.ID)
		{
		case COLOR_BYTE:
			pMat->color[0] = ReadByte();
			pMat->color[1] = ReadByte();
			pMat->color[2] = ReadByte();
			break;
		default:
			fseek(m_FilePtr, chunk.length-6, SEEK_CUR);
			break;
		}
		count += chunk.length;
	}
	return count;
}

// ��ȡ���ʵ�����
UINT C3DSModel::ReadMatMap(tMaterial *pMat, UINT n)
{
	UINT count = 0;
	tChunk chunk = {0};
	while (count<n)
	{
		ReadChunk(&chunk);
		switch (chunk.ID)
		{
		case MAP_NAME:
			ReadString(&pMat->mapName);
			pMat->isTexMat = true;
			break;
		default:
			fseek(m_FilePtr, chunk.length-6, SEEK_CUR);
			break;
		}
		count += chunk.length;
	}
	return count;
}
//
//// ����3dsģ��
//void C3DSModel::Render(void)
//{
//	tMaterial *mat;
//	t3DObject *obj;
//	int		  *index;
//	
//	for (int nOfObj=0; nOfObj<m_3DModel.numOfObjects; nOfObj++)
//	{
//		obj = &m_3DModel.pObject[nOfObj];
//		for (int nOfFace=0; nOfFace<obj->numOfFaces; nOfFace++)
//		{
//			index = obj->pFaces[nOfFace].vertIndex;
//			mat  = &m_3DModel.pMaterials[obj->pFaces[nOfFace].matID];
//			if (mat->isTexMat)				// ������Ӧ�Ĳ��ʾ�������
//			{
//				glEnable(GL_TEXTURE_2D);
//				glBindTexture(GL_TEXTURE_2D,mat->texureId);		// ѡ��ò��ʵ�����
//				glColor3ub(mat->color[0], mat->color[1], mat->color[2]);
//					if(textga.texID!=NULL)
//				glBindTexture(GL_TEXTURE_2D, textga.texID);
//				// ������������
//				glBegin(GL_TRIANGLES);
//					glTexCoord2f(obj->pTexVerts[index[0]].x,obj->pTexVerts[index[0]].y);
//					glNormal3f(obj->pNormals[index[0]].x,obj->pNormals[index[0]].y,obj->pNormals[index[0]].z);
//					glVertex3f(obj->pVerts[index[0]].x, obj->pVerts[index[0]].y, obj->pVerts[index[0]].z);
//
//					glTexCoord2f(obj->pTexVerts[index[1]].x,obj->pTexVerts[index[1]].y);
//					glNormal3f(obj->pNormals[index[1]].x,obj->pNormals[index[1]].y,obj->pNormals[index[1]].z);
//					glVertex3f(obj->pVerts[index[1]].x, obj->pVerts[index[1]].y, obj->pVerts[index[1]].z);
//
//					glTexCoord2f(obj->pTexVerts[index[2]].x,obj->pTexVerts[index[2]].y);
//					glNormal3f(obj->pNormals[index[2]].x,obj->pNormals[index[2]].y,obj->pNormals[index[2]].z);
//					glVertex3f(obj->pVerts[index[2]].x, obj->pVerts[index[2]].y, obj->pVerts[index[2]].z);
//				glEnd();
//			}
//			else							// ������Ӧ�Ĳ���û������
//			{
//				glDisable(GL_TEXTURE_2D);
//				glColor3ub(mat->color[0], mat->color[1], mat->color[2]);
//				// ������������
//				glBegin(GL_TRIANGLES);
//					glNormal3f(obj->pNormals[index[0]].x,obj->pNormals[index[0]].y,obj->pNormals[index[0]].z);
//					glVertex3f(obj->pVerts[index[0]].x, obj->pVerts[index[0]].y, obj->pVerts[index[0]].z);
//
//					glNormal3f(obj->pNormals[index[1]].x,obj->pNormals[index[1]].y,obj->pNormals[index[1]].z);
//					glVertex3f(obj->pVerts[index[1]].x, obj->pVerts[index[1]].y, obj->pVerts[index[1]].z);
//
//					glNormal3f(obj->pNormals[index[2]].x,obj->pNormals[index[2]].y,obj->pNormals[index[2]].z);
//					glVertex3f(obj->pVerts[index[2]].x, obj->pVerts[index[2]].y, obj->pVerts[index[2]].z);
//				glEnd();
//			}
//		}
//	}
//}

// �ͷ�3dsģ����Դ
void C3DSModel::Release(void)
{
	m_3DModel.numOfMaterials = 1;
	while (m_3DModel.pMaterials.size() != 0)
		m_3DModel.pMaterials.pop_back();
	m_3DModel.numOfObjects = 0;
	for (int nOfObj=0; nOfObj<m_3DModel.numOfObjects; nOfObj++)
	{
		delete [] m_3DModel.pObject[nOfObj].pFaces;
		delete [] m_3DModel.pObject[nOfObj].pVerts;
		delete [] m_3DModel.pObject[nOfObj].pTexVerts;
		delete [] m_3DModel.pObject[nOfObj].pNormals;
	}
	m_3DModel.pObject.clear();
}

// �����������Ĳ��
MyVector3 C3DSModel::Cross(MyVector3 v1, MyVector3 v2)
{
	MyVector3 vCross;

	vCross.x = ((v1.y * v2.z) - (v1.z * v2.y));
	vCross.y = ((v1.z * v2.x) - (v1.x * v2.z));
	vCross.z = ((v1.x * v2.y) - (v1.y * v2.x));

	return vCross;
}

// ������λ��
MyVector3 C3DSModel::Normalize(MyVector3 vNormal)
{
	double Magnitude;

	Magnitude = sqrt(vNormal.x*vNormal.x + vNormal.y*vNormal.y + vNormal.z*vNormal.z);
	vNormal = vNormal/(float)Magnitude;

	return vNormal;
}

// ���㶥�㷨����
void C3DSModel::ComputeNormals(void)
{
	MyVector3 v1,v2, vNormal,vPoly[3];

	// ���û��3ds������ֱ�ӷ���
	if (m_3DModel.numOfObjects <= 0)
		return;

	t3DObject *obj;
	int		  *index;

	for(int nOfObj=0; nOfObj<m_3DModel.numOfObjects; nOfObj++)
	{
		obj = &m_3DModel.pObject[nOfObj];
		MyVector3 *pNormals		= new MyVector3 [obj->numOfFaces];
		MyVector3 *pTempNormals	= new MyVector3 [obj->numOfFaces];
		obj->pNormals			= new MyVector3 [obj->numOfVerts];

		for(int nOfFace=0; nOfFace<obj->numOfFaces; nOfFace++)
		{
			index = obj->pFaces[nOfFace].vertIndex;
			// �����ε�3������
			vPoly[0] = obj->pVerts[index[0]];
			vPoly[1] = obj->pVerts[index[1]];
			vPoly[2] = obj->pVerts[index[2]];
			// ������������εķ�����
			v1 = vPoly[0]-vPoly[1];
			v2 = vPoly[2]-vPoly[1];
			vNormal  = Cross(v1, v2);

			pTempNormals[nOfFace] = vNormal;					// ����δ��λ���ķ�����
			vNormal  = Normalize(vNormal);						// ��λ��������
			pNormals[nOfFace] = vNormal;						// ���ӵ������������б�
		}
		MyVector3 vSum(0.0, 0.0, 0.0);
		MyVector3 vZero(0.0, 0.0, 0.0);
		int shared=0;

		for (int nOfVert = 0; nOfVert < obj->numOfVerts; nOfVert++)			// �������ж���
		{
			for (int nOfFace = 0; nOfFace < obj->numOfFaces; nOfFace++)		// ���������ö������
			{
				if (obj->pFaces[nOfFace].vertIndex[0] == nOfVert || 
					obj->pFaces[nOfFace].vertIndex[1] == nOfVert || 
					obj->pFaces[nOfFace].vertIndex[2] == nOfVert)
				{
					vSum = vSum+pTempNormals[nOfFace];
					shared++;
				}
			}      
			
			obj->pNormals[nOfVert] = vSum/float(-shared);

			obj->pNormals[nOfVert] = Normalize(obj->pNormals[nOfVert]);	

			vSum = vZero;
			shared = 0;
		}
	
		delete [] pTempNormals;
		delete [] pNormals;
	}

}

