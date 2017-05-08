// Loader.cpp: implementation of the Loader class.
//
//////////////////////////////////////////////////////////////////////

#include "Loader.h"
//#include "Texture.h"

// 构造函数
C3DSModel::C3DSModel()
{
	// 初始化文件指针
	m_FilePtr = NULL;

	// 定义一个默认的材质（灰色）
	tMaterial defaultMat;
	defaultMat.isTexMat = false;
	strcpy(defaultMat.matName.string, "5DG_Default");
	defaultMat.color[0] = 192;
	defaultMat.color[1] = 192;
	defaultMat.color[2] = 192;
	m_3DModel.pMaterials.push_back(defaultMat);

	// 初始化保存3DS模型的结构体
	m_3DModel.numOfMaterials = 1;
	m_3DModel.numOfObjects = 0;
}

// 析构函数
C3DSModel::~C3DSModel()
{
	m_3DModel.pMaterials.clear();
	m_3DModel.pObject.clear();
}

// 载入3ds文件
BOOL C3DSModel::Load(const char *strFileName)
{
	char strMessage[128] = {0};
	tChunk chunk = {0};

	// 打开文件
	m_FilePtr = fopen(strFileName,"rb");

	// 如果文件打开失败
	if (!m_FilePtr)
	{
		sprintf(strMessage, "3DS文件 %s 不存在！", strFileName);
		//MessageBox(NULL, strMessage, "Error", MB_OK);
		return false;
	}

	// 读取3ds文件的第一个Chunk
	ReadChunk(&chunk);

	// 检查是否是3ds文件
	if (chunk.ID != PRIMARY)
	{
		sprintf(strMessage, "读取文件 %s 失败！", strFileName);
		//MessageBox(NULL, strMessage, "Error", MB_OK);
		fclose(m_FilePtr);
		return false;
	}

	// 开始读取3ds文件
	ReadPrimary(chunk.length-6);

	// 计算每个顶点的法线量
	//ComputeNormals();

	// 关闭打开的文件
	fclose(m_FilePtr);
	m_FilePtr = NULL;

	// 对有纹理的材质载入该纹理
	//for (int i=0; i<m_3DModel.numOfMaterials; i++)
	//{
	//	if (m_3DModel.pMaterials[i].isTexMat)
	//	{
	//		if (!BuildTexture(m_3DModel.pMaterials[i].mapName.string, m_3DModel.pMaterials[i].texureId))
	//		{
	//			// 纹理载入失败
	//			if(!BuildTexture(m_3DModel.pMaterials[i].mapName.string, &textga))
	//			{
	//			sprintf(strMessage, "3DS纹理文件载入失败: %s ！", m_3DModel.pMaterials[i].mapName.string);
	//			MessageBox(NULL, strMessage, "Error", MB_OK);
	//			}
	//		}
	//	}
	//}

	return true;
}

// 从文件中读取1个字节
BYTE C3DSModel::ReadByte(void)
{
	BYTE result = 0;
	fread(&result, 1, 1, m_FilePtr);
	return result;
}

// 从文件中读取2个字节
WORD C3DSModel::ReadWord(void)
{
	return ReadByte() + (ReadByte()<<8);
}

// 从文件中读取4个字节
UINT C3DSModel::ReadUint(void)
{
	return ReadWord() + (ReadWord()<<16);
}

// 从文件中读取浮点数
float C3DSModel::ReadFloat(void)
{
	float result;
	fread(&result, sizeof(float), 1, m_FilePtr);
	return result;
}

// 从文件中读取字符串（返回字符串长度）
UINT C3DSModel::ReadString(STRING *pStr)
{
	int n=0;
	while ((pStr->string[n++]=ReadByte()) != 0)
		;
	return n;
}

// 读取3ds的一个Chunk信息
void C3DSModel::ReadChunk(tChunk *pChunk)
{
	fread(&pChunk->ID, 1, 2, m_FilePtr);
	fread(&pChunk->length, 1, 4, m_FilePtr);
}

// 读取3ds文件主要Chunk
UINT C3DSModel::ReadPrimary(UINT n)
{
	UINT count = 0;				// 该Chunk内容已读取的字节计数
	tChunk chunk = {0};			// 用以保存子Chunk的内容
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

// 读取3ds物体主编辑Chunk
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

// 读取3ds对象
UINT C3DSModel::ReadObject(UINT n)
{
	UINT count = 0;
	tChunk chunk = {0};
	// 新的3ds对象
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
	// 保存3ds对象
	m_3DModel.pObject.push_back(newObject);
	return count;
}

// 读取3ds对象信息
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
			// 按块读取顶点坐标值
			fread(pObj->pVerts, 1, chunk.length - 8, m_FilePtr);
			// 调换y、z坐标值(由于3dMAX坐标系方向与OpenGL不同)
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
			// 按块读取纹理坐标值
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

// 读取面信息
UINT C3DSModel::ReadFacetInfo(t3DObject *pObj, UINT n)
{
	UINT count = 0;
	tChunk chunk = {0};
	pObj->numOfFaces = ReadWord();
	pObj->pFaces = new tFace[pObj->numOfFaces];
	memset(pObj->pFaces, 0, sizeof(tFace) * pObj->numOfFaces);
	// 读取面索引值(第4个值为3dMAX使用的参数，舍弃)
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
			ReadString(&matName);			// 材质名称
			strcpy(pObj->matName.string,matName.string);	//保存材质的名称
			t=ReadWord();					// 材质对应的面个数
			// 查找对应的材质
			for ( i=1;i<=m_3DModel.numOfMaterials;i++)
			{
				if (strcmp(matName.string, m_3DModel.pMaterials[i].matName.string) == 0)
				{
					matID = i;
					break;
				}
			}
			// 依据面索引给每个面绑定材质ID
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

// 读取材质
UINT C3DSModel::ReadMaterial(UINT n)
{
	UINT count = 0;
	tChunk chunk = {0};
	// 新的材质
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
	// 保存新的材质
	m_3DModel.pMaterials.push_back(newMaterial);
	return count;
}

// 读取材质的漫反射属性
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

// 读取材质的纹理
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
//// 绘制3ds模型
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
//			if (mat->isTexMat)				// 如果面对应的材质具有纹理
//			{
//				glEnable(GL_TEXTURE_2D);
//				glBindTexture(GL_TEXTURE_2D,mat->texureId);		// 选择该材质的纹理
//				glColor3ub(mat->color[0], mat->color[1], mat->color[2]);
//					if(textga.texID!=NULL)
//				glBindTexture(GL_TEXTURE_2D, textga.texID);
//				// 绘制三角形面
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
//			else							// 如果面对应的材质没有纹理
//			{
//				glDisable(GL_TEXTURE_2D);
//				glColor3ub(mat->color[0], mat->color[1], mat->color[2]);
//				// 绘制三角形面
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

// 释放3ds模型资源
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

// 计算两向量的叉积
MyVector3 C3DSModel::Cross(MyVector3 v1, MyVector3 v2)
{
	MyVector3 vCross;

	vCross.x = ((v1.y * v2.z) - (v1.z * v2.y));
	vCross.y = ((v1.z * v2.x) - (v1.x * v2.z));
	vCross.z = ((v1.x * v2.y) - (v1.y * v2.x));

	return vCross;
}

// 向量单位化
MyVector3 C3DSModel::Normalize(MyVector3 vNormal)
{
	double Magnitude;

	Magnitude = sqrt(vNormal.x*vNormal.x + vNormal.y*vNormal.y + vNormal.z*vNormal.z);
	vNormal = vNormal/(float)Magnitude;

	return vNormal;
}

// 计算顶点法线量
void C3DSModel::ComputeNormals(void)
{
	MyVector3 v1,v2, vNormal,vPoly[3];

	// 如果没有3ds对象则直接返回
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
			// 三角形的3个顶点
			vPoly[0] = obj->pVerts[index[0]];
			vPoly[1] = obj->pVerts[index[1]];
			vPoly[2] = obj->pVerts[index[2]];
			// 计算这个三角形的法线量
			v1 = vPoly[0]-vPoly[1];
			v2 = vPoly[2]-vPoly[1];
			vNormal  = Cross(v1, v2);

			pTempNormals[nOfFace] = vNormal;					// 保存未单位化的法向量
			vNormal  = Normalize(vNormal);						// 单位化法向量
			pNormals[nOfFace] = vNormal;						// 增加到法向量数组列表
		}
		MyVector3 vSum(0.0, 0.0, 0.0);
		MyVector3 vZero(0.0, 0.0, 0.0);
		int shared=0;

		for (int nOfVert = 0; nOfVert < obj->numOfVerts; nOfVert++)			// 遍历所有顶点
		{
			for (int nOfFace = 0; nOfFace < obj->numOfFaces; nOfFace++)		// 遍历包含该顶点的面
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

