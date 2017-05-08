#ifndef WRITER_H
#define WRITER_H
#include "Macros3ds.h"
#include "Loader.h"
#include <iostream>

class C3DWriter: public C3DSModel 
{
public:
	C3DWriter();
	C3DWriter(string file_name);
	~C3DWriter();
	void write(string file_name);
	void output(ostream& out);
public:
	bool WriteByte(BYTE);
	bool WriteWord(WORD);
	bool WriteUint(UINT);
	bool WriteFloat(float);
	bool WriteString(STRING& str);
	
	void WriteChunk(tChunk chunk);
	void WritePrimChunk(t3DModel *pModel, FILE* outf);
	int WriteMatrial(tMaterial *pMat,FILE* outf,int n);
	int WriteMeshObj(t3DObject *pObj, FILE* outf, int n);

public:
	FILE *m_outFilePtr;								// 3dsÎÄ¼þÖ¸Õë	
public:
	void testWrite(string file_name);				// 

};

#endif