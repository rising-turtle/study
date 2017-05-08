#include "stdio.h"
#include "stdlib.h"

#define pi 3.141592654
#define erose_area 50

double square(double x, double y)
{
	return x*x+y*y;
} 

int calculateyear(double dis)
{
	int i;
	double delta = 2*erose_area/pi;
	i = dis/delta;
	if(i*delta>=dis)
		return i;
	return i+1;
}

int main1005()
{
	int num;
	int i;
	double dx,dy;
	scanf("%d",&num);
	for(i=0;i<num;i++){
		scanf("%lf %lf",&dx,&dy);
		printf("Property %d: This property will begin eroding in year %d.\n",i+1,calculateyear(square(dx,dy)));
	}
	printf("END OF OUTPUT.\n");
	return 0;
}
