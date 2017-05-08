#include <iostream>
#include <vector>
#include <algorithm>
#include "stdlib.h"

using namespace std;

static const unsigned int N = 10;

int gen20(){
	static bool first = true;
	if(first){
		first = false;
		srand(time(NULL));
	}
	return rand()%20;
}

void swap(int * a, int l, int r)
{
	int t =*(a+l);
	*(a+l) = *(a+r);
	*(a+r) = *(a+l);
}

void qsort(int* a, int l, int h)
{
	// cout<<"l= "<<l<<" h= "<<h<<endl;
	if(l>=h-1) return ;
	int index = rand()%(h-l) + l;
	swap(a,l,index);
	int key =*(a+l);
	int i=1;
	for(int k=l+1;k<h;k++)
	{
		int v = *(a+k);
		if(v >= key) continue;
		*(a+k) = *(a+l+i);
		*(a+l+i) = v;
		i++;
	}
	*(a+l) = *(a+l+i-1);
	*(a+l+i-1) = key;
	qsort(a,l,l+i-1);
	qsort(a,l+i,h);
}

void show(int * a, int n){
	for(int i=0;i<n;i++)
		cout<<*(a+i)<<" ";
	cout<<endl;
}


int main()
{
	int array[N];
	srand(time(NULL));
	generate(array,array+N,gen20);
	show(array,N);
	qsort(array,0,N);
	show(array,N);
	return 0;
}
