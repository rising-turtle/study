#include "preheader.h"

static int cnt =0;

void Swap(vector<int>& swap_, int l, int r){
	int tmp = swap_[l];
	swap_[l] = swap_[r];
	swap_[r] = tmp;
}

// Full sort for (vector& ,index)  
void fullsort(vector<int>& raw, vector<int>&sorted, int index){
	if(index == raw.size()-1)
	{
		sorted[index] = raw[index];
		cnt++;
		for(int i=0;i<=index;i++)
			cout<<sorted[i]<<" ";
		cout<<endl;
	}
	for(int i=index;i<raw.size();i++)
	{
		Swap(raw,index,i);
		sorted[index] = raw[index];
		fullsort(raw,sorted,index+1);
		Swap(raw,i,index);
	}
	return ;
}
// 
void test_fullsort(int N){
	vector<int> raw(N);
	vector<int> sorted(N);
	for(size_t i=0;i<N ;i++){
		raw[i] = i;
	}
	fullsort(raw,sorted,0);
	cout<<"total :"<<cnt<<endl;
}
//
//void main()
//{
//	test_fullsort(5);
//	getchar();
//}