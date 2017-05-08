#include "stdio.h"
#include "stdlib.h"

double min_hang=0.01;
double max_hang=5.20;
double sumlen(int n);

int findmaxInt()
{
	int ret=0;
	int i;
	for(i=1;;i++){
	   if(sumlen(i)>=max_hang)
		break;
	}
	return i;
}

double sumlen(int n){
	if(n<=0) return 0;
	if(n==1) return 0.5;
	return sumlen(n-1)+1./(n+1);
}

int findNCards(double target,int min_card, int max_card){
	if(min_card>=max_card) return min_card;
	int mid=min_card+(max_card-min_card)/2;
	double mid_hang = sumlen(mid);
	if(mid_hang>=target){
		if(mid==min_card || sumlen(mid-1)<target )
			return mid;
		else
		  return findNCards(target,min_card,mid-1);
	}
	else{
		if(mid==max_card || sumlen(mid+1)>=target)
			return mid+1;
		else
		  return findNCards(target,mid+1,max_card);
	}
	// not possible
	return -1;
}

int main(){
	double overhang;
	int min_card=1;
	int max_card=findmaxInt();
	while(scanf("%lf",&overhang)!=EOF){
		if(overhang<min_hang || overhang>max_hang)
		{
			printf("exceed range\n");
			continue;
		}
		printf("%d\n",findNCards(overhang,min_card,max_card));
	}	
	return 0;
}
