#include "stdio.h"
#include "stdlib.h"

double expetional(double a, int cout){
	double ret=1;
	double multi=a;
	int i;
	if(cout==0)
		return 1;
	if(cout<0){
		multi = 1./a;
		cout*=-1;
	}
	for(i=0;i<cout;i++){
		ret*=multi;
	}
	return ret;
}

int main(){
	char buf[100];
	int cout=0;
	while(scanf("%s %d",buf,&cout)!=EOF){
		double d = atof(buf);
		printf("%lf\n",expetional(d,cout));
		//printf("d=%lf,cout=%d\n",d,cout);
	}
	return 0;
}
