#include "preheader.h"

template<typename T>
void display(vector<T>& v)
{
	vector<T>::iterator it=v.begin();
	while(it!=v.end())
	{
		cout<<(*it)<<" ";
		it++;
	}
	cout<<endl;
}

template<>
void display(vector<int>& v)
{
	for(size_t i=0;i<v.size();i++)
	{
		cout<<v[i]<<" ";
	}
	cout<<endl;
}