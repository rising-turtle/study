#include "preheader.h"

#define RX 2000 // X [-10 10]
#define RY 600	// Y [-3 3]
#define RZ 2000 // Z [-10 10]

#define CELLSIZE 4
#define X_CELL RX/CELLSIZE // 400
#define Y_CELL RY/CELLSIZE // 120
#define Z_CELL RZ/CELLSIZE // 400

#define X_STEP Y_CELL*Z_CELL // 120*400
#define Y_STEP Z_CELL // 400

#define ALL_CELLS X_CELL*Y_CELL*Z_CELL // 

int getIndexCell(float& x,float& y, float& z){
	/*if(!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
		return -1;*/
	if(fabs(x) > 10 || fabs(z) > 10 || y<-3 || y>3 )
		return -1;
	int lx = ( (int)(x*100) + RX/2) / CELLSIZE;
	int ly = ( (int)(y*100) + RY/2) / CELLSIZE;
	int lz = ( (int)(z*100) + RZ/2) / CELLSIZE;
	return (lx*X_STEP + ly*Y_STEP + lz);
}

void testHash()
{
	fstream filelog;
	filelog.open("D:\\testHASH.log",ios::out);
	int cnt=0;
	int index = 0;
	int getindex;

	float step = (float)(CELLSIZE)/100.0;
	for(float x= -10.0; x<10.0; x+= step)
		for(float y= -3.0;y<3.0; y+=step)
			for(float z=-10.0;z<10.0;z+=step){
				getindex = getIndexCell(x,y,z);
				if(index != getindex){
					cout<<"(x,y,z):("<<x<<","<<y<<","<<z<<") :"<<"getindex="<<getindex<<"; index="<<index<<endl;
					++cnt;
				}
				index++;
			}
			cout<<"error: "<<cnt<<endl;
}

void main()
{
	testHash();
}