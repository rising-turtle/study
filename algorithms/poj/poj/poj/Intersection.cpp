#include "preheader.h"
#include "stdlib.h"
#include <ctime>
#include <algorithm>
namespace {
vector<int> intersection(vector<int> a,vector<int> b)
{
	vector<int> ret;
	int ia,ib;
	for(ia=0,ib=0;ia<a.size() && ib<b.size(); )
	{
		if(a[ia] == b[ib]){
			ret.push_back(a[ia]);
			while(a[++ia]==b[++ib])  ret.push_back(a[ia]);
			continue;
		}
		else if(a[ia]<b[ib])
		{
			while(ia<a.size() && a[ia]<b[ib]) ia++;
			continue;
		}
		else{
			while(ib<b.size() && b[ib]<a[ia]) ib++;
			continue;
		}
	}
	return ret;
}

std::ostream& operator<<(std::ostream& out,vector<int> tmp)
{
	for(int i=0;i<tmp.size();i++)
		out<<tmp[i]<<" ";
	out<<std::endl;
	return out;
}
}
void test_intersection(){
	vector<int> a,b,c;
	srand((unsigned int)(time(NULL)));
	int N=10;
	for(int i=0;i<N;i++){
		a.push_back(rand()%N);
		b.push_back(rand()%N);
	}
	std::sort(a.begin(),a.end());
	std::sort(b.begin(),b.end());
	c=intersection(a,b);
	std::cout<<"a: "<<a<<"b: "<<b<<"c: "<<c;
}
