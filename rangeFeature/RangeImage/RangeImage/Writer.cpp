#include "Macros3ds.h"
#include "Writer.h"
#include <iostream>
#include <assert.h>

using namespace std;

C3DWriter::C3DWriter(){}
C3DWriter::C3DWriter(string file_name)
{
	char tmpfile[128];
	strcpy(tmpfile,file_name.c_str());
	//Load(file_name.c_str());
	Load(tmpfile);
}
C3DWriter::~C3DWriter(){
	Release();
}

void C3DWriter::write(string file_name)
{
	m_outFilePtr=fopen(file_name.c_str(),"wb");
	if(!m_outFilePtr){
		cout<<"error to open file: "<<file_name<<endl;
		return ;
	}
	WritePrimChunk(&m_3DModel,m_outFilePtr);
	fclose(m_outFilePtr);
}

bool C3DWriter::WriteByte(BYTE m)
{
	if(fwrite(&m,1,1,m_outFilePtr)==1)
		return true;
	return false;
}

bool C3DWriter::WriteWord(WORD w)
{
	BYTE um=(w&0xFF00)>>8;
	BYTE lm=w&0x00FF;
	return (WriteByte(lm) && WriteByte(um));
}

bool C3DWriter::WriteUint(UINT u)
{
	WORD uw=(u&0xFFFF0000)>>16;
	WORD lw=u&0x0000FFFF;
	return (WriteWord(lw) && WriteWord(uw));
}

bool C3DWriter::WriteFloat(float f)
{
	if((fwrite(&f,sizeof(float),1,m_outFilePtr))!=sizeof(float))
		return false;
	return true;
}
bool C3DWriter::WriteString(STRING& str)
{
	int i=0;
	while(str.string[i]!='\0')
	{
		if(WriteByte(str.string[i])==false)
			return false;
		i++;
	}
	WriteByte(str.string[i]);
	return true;
}

void C3DWriter::WriteChunk(tChunk chunk)
{
	WriteWord(chunk.ID);
	WriteUint(chunk.length);
}

void C3DWriter::testWrite(string file_name)				// 
{
	m_outFilePtr=fopen(file_name.c_str(),"w");
	tChunk prim;
	prim.ID=PRIMARY;
	prim.length=20;
	WriteChunk(prim);
	tChunk version;
	version.ID=M3D_VERSION;
	version.length=10;//prim.length-6;
	WriteChunk(version);
	unsigned int version_v=3;
	WriteUint(version_v);

	fclose(m_outFilePtr);
	cout<<"wait to read file_name: "<<file_name<<endl;
	getchar();
	m_FilePtr=fopen(file_name.c_str(),"rb");
	tChunk tmp;
	ReadChunk(&tmp);
	cout<<"prim ID: "<<tmp.ID<<" Len: "<<tmp.length<<endl;
	ReadChunk(&tmp);
	cout<<"vern ID: "<<tmp.ID<<" Len: "<<tmp.length<<endl;
	cout<<"Mesh version: "<<ReadUint()<<endl;
	return ;
}

void C3DWriter::WritePrimChunk(t3DModel *pModel, FILE* outf)
{
	tChunk prim;
	prim.ID=PRIMARY;
	prim.length=pModel->size();
	WriteChunk(prim);
	tChunk version;
	version.ID=M3D_VERSION;
	version.length=10;//prim.length-6;
	WriteChunk(version);
	unsigned int version_v=3;
	WriteUint(version_v);
	tChunk prim_edit;
	prim_edit.ID=PRIM_EDIT;
	prim_edit.length=prim.length-16;
	WriteChunk(prim_edit);
	tChunk mesh_version;
	mesh_version.ID=MESH_VERN;
	mesh_version.length=10;//prim_edit.length-6;
	WriteChunk(mesh_version);
	WriteUint(version_v);
	int cur_size=pModel->size()-32;
	for(size_t i=1;i<pModel->pMaterials.size();i++)
	{
		int matsize=pModel->pMaterials[i].size();
		int writen=WriteMatrial(&(pModel->pMaterials[i]),outf,matsize);
		assert(writen==matsize);
		cur_size-=writen;
	}
	for(size_t i=0;i<pModel->pObject.size();i++)
	{
		int objsize=pModel->pObject[i].size();
		int writen=WriteMeshObj(&(pModel->pObject[i]),outf,objsize);
		assert(writen==objsize);
		cur_size-=writen;
	}
	assert(cur_size==0);
}
int C3DWriter::WriteMatrial(tMaterial *pMat,FILE* outf, int n)
{
	int cout=0;
	tChunk mat;						// 0xafff 写入mat
	mat.ID=EDIT_MAT;
	mat.length=n;
	WriteChunk(mat);
	cout+=6;
	tChunk mat_name;				// mat名称
	mat_name.ID=MAT_NAME;
	mat_name.length=pMat->matName.size()+6;//mat.length-6;
	WriteChunk(mat_name);
	cout+=6;
	WriteString(pMat->matName);
	cout+=pMat->matName.size();
	tChunk mat_map;					// 材质纹理
	mat_map.ID=MAT_MAP;
	mat_map.length=pMat->mapName.size()+12;//mat_name.length-6-pMat->matName.size();
	WriteChunk(mat_map);
	cout+=6;
	tChunk map_name;				// 纹理名称
	map_name.ID=MAP_NAME;
	map_name.length=pMat->mapName.size()+6;//mat_map.length-6;
	WriteChunk(map_name);
	cout+=6;
	WriteString(pMat->mapName);
	cout+=pMat->mapName.size();
	return cout;
}
int C3DWriter::WriteMeshObj(t3DObject *pObj, FILE* outf, int n)
{
	int cout=0;
	tChunk mesh_edit;				// 网格名字
	mesh_edit.ID=EDIT_OBJECT;
	mesh_edit.length=n;
	std::cout<< "mesh edit length: "<< mesh_edit.length<<std::endl;
	WriteChunk(mesh_edit);
	cout+=6;
	WriteString(pObj->objName);
	cout+=pObj->objName.size();

	tChunk mesh_info;				// 网格信息
	mesh_info.ID=OBJECT_INFO;
	mesh_info.length=mesh_edit.length-6-pObj->objName.size();
	std::cout<<"mesh info length: "<<mesh_info.length<<std::endl;
	WriteChunk(mesh_info);
	cout+=6;

	tChunk mesh_vertex;				// 顶点坐标信息
	mesh_vertex.ID=OBJECT_VERTEX;
	mesh_vertex.length=pObj->vertex_size();
	std::cout<<"mesh_vertex length: "<<mesh_vertex.length<<std::endl;
	WriteChunk(mesh_vertex);
	WriteWord(pObj->numOfVerts);
	cout+=8;
	for(int i=0;i<pObj->numOfVerts;i++)
	{
		float u,v,w;
		MyVector3& pver=pObj->pVerts[i];
		u=pver.x;
		v=-pver.z;
		w=pver.y;
		WriteFloat(u);
		WriteFloat(v);
		WriteFloat(w);
	}
	cout+=pObj->numOfVerts*12;

	tChunk mesh_texture;			//顶点纹理信息
	mesh_texture.ID=OBJECT_UV;
	mesh_texture.length=pObj->mat_size();
	std::cout<<"mesh_texture length: "<<mesh_texture.length<<std::endl;
	WriteChunk(mesh_texture);
	WriteWord(pObj->numOfVerts);
	cout+=8;
	for(int i=0;i<pObj->numTexVertex;i++)
	{
		MyVector2& ptex=pObj->pTexVerts[i];
		WriteFloat(ptex.x);
		WriteFloat(ptex.y);
	}
	cout+=8*pObj->numTexVertex;
	
	tChunk mesh_local;				// 旋转矩阵
	mesh_local.ID=OBJECT_LOCAL;
	mesh_local.length=4*12+6;
	WriteChunk(mesh_local);
	cout+=6;
	float matrix[16]={0};
	memset(matrix,0,64);
	matrix[0]=matrix[5]=matrix[10]=matrix[15]=1.0;
	for(int i=0;i<4;i++)
		for(int j=0;j<3;j++)
	{
		WriteFloat(matrix[i*4+j]);
	}
	cout+=48;

	tChunk mesh_facet;				// 面信息
	mesh_facet.ID=OBJECT_FACET;
	mesh_facet.length=pObj->facet_size();
	std::cout<<"mesh_facet length: "<<mesh_facet.length<<std::endl;
	WriteChunk(mesh_facet);
	WriteWord(pObj->numOfFaces);
	cout+=8;
	
	for(int i=0;i<pObj->numOfFaces;i++)
	{
		tFace& pface=pObj->pFaces[i];
		WriteWord(pface.vertIndex[0]);
		WriteWord(pface.vertIndex[1]);
		WriteWord(pface.vertIndex[2]);
		WriteWord(1);				// 面信息，表示映射的方向等
	}
	cout+=pObj->numOfFaces*8;

	tChunk mesh_map;				// 材质信息
	mesh_map.ID=FACET_MAT;
	mesh_map.length=pObj->map_size();
	WriteChunk(mesh_map);
	WriteString(pObj->matName);
	WriteWord(pObj->numOfFaces);
	cout= cout + 8 + pObj->matName.size();
	
	for(int i=0;i<pObj->numOfFaces;i++)
		WriteWord(i);
	cout+=2*pObj->numOfFaces;

	return cout;
}

void C3DWriter::output(ostream& out)
{
	out<<"objects: "<<m_3DModel.numOfObjects<<endl;
	out<<"materials: "<<m_3DModel.numOfMaterials<<endl;
	for(int i=1;i<m_3DModel.numOfMaterials;i++)
	{
		tMaterial& mMat=m_3DModel.pMaterials[i];
		out<<"mat "<<i+1<<":"<<endl;
		string tmps(mMat.matName.string);
		string tmpms(mMat.mapName.string);
		out<<"mat_name: "<<tmps<<", mat_map: "<<tmpms<<endl;
	}
	for(int i=0;i<m_3DModel.numOfObjects;i++)
	{
		t3DObject& mobj=m_3DModel.pObject[i];
		out<<"obj "<<i+1<<":"<<endl;
		out<<"vertexs: "<<mobj.numOfVerts<<endl;
		out<<"texverts: "<<mobj.numTexVertex<<endl;
		out<<"facets: "<<mobj.numOfFaces<<endl;
		if(mobj.numOfFaces<=0)
			continue;
		tFace& curface =mobj.pFaces[0];
		if(curface.matID>m_3DModel.pMaterials.size() || curface.matID<0)
		{
			continue;
		}
		string tmps(m_3DModel.pMaterials[mobj.pFaces[0].matID].matName.string);
		out<<"texture: "<<tmps<<endl;
	}
}