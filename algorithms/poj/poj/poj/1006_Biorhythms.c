/*
	解同余方程组 
	N % a1 = r1
	N % a2 = r2
	N % a3 = r3
	1, find g1,g2,g3 
	   g1 = k*a2*a3 that g1%a1 = 1
	   g2 = k*a1*a3 that g2%a2 = 1
	   g3 = k*a1*a2 that g3%a3 = 1
    2, then ret = r1*g1 + r2*g2 + r3*g3 = minN + k*a1*a2*a3
*/
#include "stdio.h"
#include "stdlib.h"

static int div1 = 23;
static int div2 = 28;
static int div3 = 33;

int m1,m2,m3;

void findRestOneNumber(int* f1,int* f2,int* f3)
{
	int i;
	int p1 = div2*div3;
	int p2 = div1*div3;
	int p3 = div1*div2;
	int g1=0;
	int g2=0;
	int g3=0;
        for(i=1;;i++)
	{
	    if(!g1&&(i*p1)%div1==1){*f1=p1*i;g1=1;}
	    if(!g2&&(i*p2)%div2==1){*f2=p2*i;g2=1;}
	    if(!g3&&(i*p3)%div3==1){*f3=p3*i;g3=1;}
	    if(g1 && g2 && g3)
		return ;
	}
	return ;
}

int getNextTriplePeak(int r1, int r2, int r3, int d)
{
	static int shared_m = 21252;//div1*div2*div3;
	int ret = r1*m1 + r2*m2 + r3*m3;
	int rest = ret-(ret/shared_m)*shared_m;
	//while(rest>0)
	//{
	//	rest -= shared_m;
	//}
	//rest+=shared_m;
	rest-=d;
	if(rest<=0)
	   rest+=shared_m;
	return rest;
}

int main1006()
{
    int r1,r2,r3;
    int d;
    findRestOneNumber(&m1,&m2,&m3);
    printf("g1: %d, g2: %d, g3: %d\n",m1,m2,m3);	
    while(scanf("%d %d %d %d",&r1,&r2,&r3,&d)!=EOF)
    {
       static int case_n=1;
       if(r1==-1) break;
       printf("Case %d: the next triple peak occurs in %d days.\n",case_n++,getNextTriplePeak(r1,r2,r3,d));
    }
return 0;
}
