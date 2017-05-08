#include "NormalVector.h"

CNormalVector::CNormalVector():nx(1),ny(0),nz(0){}
CNormalVector::CNormalVector(float x,float y, float z):nx(x),ny(y),nz(z){
	Normalization();
}
CNormalVector::~CNormalVector(){}

void CNormalVector::Normalization()
{
	//if(x>0){x*=-1.f; y*=-1.f; z*=-1.f;}
	float d = nx*nx + ny*ny + nz*nz;
	d = sqrt((double)d);
	nx /=d; ny/=d; nz/=d; 
}
void CNormalVector::GetNormal(float& x,float& y,float& z){
	x=nx; y=ny; z=nz;
}
