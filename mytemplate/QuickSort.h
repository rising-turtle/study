#ifndef MYQUICKSORT_H
#define MYQUICKSORT_H

#include <vector>
#include <iostream>
using std::vector;
using std::cout;

template<typename T>
void QuickSort(vector<T> &s, vector<int>& index)
{
	size_t N=s.size();
	if(index.size()!=N)
		index.resize(N);
	for(size_t i=0;i<N;i++)
		index[i]=i;
	int low=0;
	int high=N;
	QuickSortLoop(s,index,low,high);
}
// QuickSort Area [low,high)
template<typename T>
void QuickSortLoop(vector<T>& s,vector<int>& index, int low, int high)
{
	if(low>=high) return;
	int i=low;
	int j=high;
	T pivot=s[index[low]];
	while(i<j)
	{
		if(!(pivot<s[index[i]])) i++;
		else // pivot < s[index[i]] 
		{
			int tmp=index[i];
			index[i]=index[j-1];
			index[j-1]=tmp;
			j--;
		}
	}
	int tmp=index[low];
	index[low]=index[i-1];
	index[i-1]=tmp;
	QuickSortLoop(s,index,low,i-1);
	QuickSortLoop(s,index,i,high);
}

template<typename T>
void displayVec(vector<T>& s)
{
	vector<T>::iterator it=s.begin();
	while(it!=s.end()){
		cout<<*it<<" ";
		it++;
	}
	cout<<endl;
}
template<typename T>
void displayVec(vector<T>& s, vector<int>& index)
{
	size_t N=s.size();
	for(size_t i=0;i<index.size();i++)
	{
		int e_index=index[i];
		if(e_index>=N)
		{
			cout<<"exceed range of s, while index="<<e_index<<endl;
			return;
		}
		cout<<s[e_index]<<" ";
	}
	cout<<endl;
}
#endif