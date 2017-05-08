#include <algorithm>
#include <vector>
#include <iostream>
#include <cmath>

using namespace std;


int main()
{
#define N 10
	vector<float> tmp(N);
	for(int i=0;i<N;i++){
		tmp[i] = (float)(N-i)/(N+1);
		cout<<tmp[i]<<" ";
	}
	cout<<endl;
	sort(tmp.begin(),tmp.end());
	for(int i=0;i<tmp.size();i++)
		cout<<tmp[i]<<" ";
	cout<<endl;
	return 0;
}
