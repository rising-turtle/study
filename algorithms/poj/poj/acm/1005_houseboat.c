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

int main()
{
	printf("(1.0,1.0): %d\n",calculateyear(square(1.0,1.0)));
	printf("(25.0,0.0): %d\n",calculateyear(square(25.0,0.0)));
	return 0;
}
