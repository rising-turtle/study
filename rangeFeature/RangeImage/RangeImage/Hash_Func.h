#ifndef HASHFUNC_H
#define HASHFUNC_H
#include "preheader.h"
#include "TriangleMesh.h"

struct hash_func1{
public:
	size_t operator()(const IPt& pt) const
	{
		int x=pt.ix; int y=pt.iy; int z=pt.iz;
		char tmpstr[12];
		memcpy(tmpstr,&x,sizeof(int));
		memcpy(tmpstr+sizeof(int),&y,sizeof(int));
		memcpy(tmpstr+2*sizeof(int),&z,sizeof(int));
		
		/* ELFHash */
		unsigned int h=0;
		unsigned int g=0;
		char* pchar=tmpstr;
		for(int i=0;i<12;i++)
		{
			h=(h << 4)+*pchar;
			pchar++;
			if(g= h & 0xf0000000)
				h^= g >> 24;
			h &= ~g;
		}
		return h;
	}
};


#endif