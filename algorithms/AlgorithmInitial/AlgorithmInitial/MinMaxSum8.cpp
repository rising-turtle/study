#include "preheader.h"

int MaxSumAt(vector<int>& integral,int location_m, int m)
{
	int ret=-10000;
	int max_before=0;
	int min_max=10000;
	if(m==0)
		return (integral[integral.size()-1]-integral[location_m]);
	for(size_t i=location_m+1;i<integral.size()-m;i++)
	{
		max_before=integral[i]-integral[location_m];
		ret=max(max_before,MaxSumAt(integral,i,m-1));
		if(min_max>ret)
			min_max=ret;
	}
	if(ret==-10000)
		ret=ret;
	return min_max;//ret;
}

int MinMaxSum(vector<int>& ia,int m)
{
	// check validity
	assert(m>0);
	assert(ia.size()>m);

	// Initialization
	vector<int> Integral;
	Integral.resize(ia.size(),0);
	Integral.assign(ia.begin(),ia.end());
	for(size_t i=1;i<ia.size();i++)
	{
		Integral[i]+=Integral[i-1];
	}

	int max_sum=-10000;
	int min_max_sum=10000;
	for(int location_m=0;location_m<Integral.size()-m;location_m++)
	{
		max_sum=max(Integral[location_m],MaxSumAt(Integral,location_m,m-1));
		if(max_sum<min_max_sum)
			min_max_sum=max_sum;
	}
	return min_max_sum;
}

void testMinMaxSum(int n,int m)
{
	vector<int> ia;
	ia.resize(n);
	srand(unsigned int(time(NULL)));
	for(int i=0;i<n;i++)
	{
		ia[i]=(rand()%10-2);
		cout<<" "<<ia[i];
	}
	cout<<endl;
	cout<<"min_max_sum:"<<MinMaxSum(ia,m)<<endl;
}

//void main()
//{
//	testMinMaxSum(6,2);
//}