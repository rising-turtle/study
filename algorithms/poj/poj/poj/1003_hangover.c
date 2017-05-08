#include "stdio.h"
#include "stdlib.h"

double gl_min_hang=0.01;
double gl_max_hang=5.20;
double sumlen(int n);

extern int main1003();

int findmaxInt()
{
	int ret=0;
	int i;
	for(i=1;;i++){
	   if(sumlen(i)>=gl_max_hang)
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
	int mid;
	double mid_hang;
	if(min_card>=max_card) return min_card;
	mid=min_card+(max_card-min_card)/2;
	mid_hang = sumlen(mid);
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

int mycards(double range){
	int min_card=1;
	int max_card=findmaxInt();
	return findNCards(range,min_card,max_card);
}


int main1003(){
	double overhang;
	int min_card=1;
	int max_card=findmaxInt();
	while(scanf("%lf",&overhang)!=EOF){
		if(overhang<gl_min_hang || overhang>gl_max_hang)
		{
			break;
			printf("exceed range\n");
			continue;
		}
		printf("%d\n",findNCards(overhang,min_card,max_card));
	}	
	return 0;
}
