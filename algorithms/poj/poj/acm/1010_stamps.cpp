#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <set>

using namespace std;

int types[4];
int type_num[4];

vector<vector<int> > valid_combine;
set<int> unique_status;

int cal_status(int sum,int index,vector<int> score)
{
	int ret=0;
	int multi=1;
	for(int i=0;i<score.size();i++)
	{	
		ret+=score[i]*multi;
		multi*=10;
	}
	return ret+=index*10000;
}

void recordThisCombine(int type_num[4])
{
	vector<int> valid_(4,0);
	for(int i=0;i<4;i++) valid_[i]=type_num[i];
	valid_combine.push_back(valid_);
	return;
}
void findValidCombine(int sum, int index, vector<int> type_)
{
	if(index>=4) return ;
	int status = cal_status(sum,index,type_);
	if(unique_status.find(status)!=unique_status.end()){ return ;}
	unique_status.insert(status);

	if(sum==0)
	{
		valid_combine.push_back(type_);
		return ;
	}
	if(types[index]==0)
	{
	        return ;
	}
	// directly turn to next 
	findValidCombine(sum,index+1,type_);
	for(int i=1;;i++)
	{
		int rest = sum-i*types[index];
		if(rest<0) return ;
		type_[index]++;
		if(rest!=0) findValidCombine(rest,index+1,type_);
		findValidCombine(rest,index,type_);
	}
	return ;
}

void displayValidComb(int sum)
{
	int n = valid_combine.size()>10?10:valid_combine.size();
	for(int i=0;i<valid_combine.size();i++)
	{
	   cout<<"sum: "<<sum<<"= ";
	   for(int j=0;j<valid_combine[i].size();j++)
		{
			cout<<valid_combine[i][j]<<"*"<<types[j]<<" ";
		}
		cout<<endl;
	}
}

int kind_score(vector<int> a)
{
	int ret=0;
	for(int i=0;i<a.size();i++)
		if(a[i]!=0)
		   ret++;
	return ret;
}

int num_score(vector<int> a)
{
	int ret=0;
	for(int i=0;i<a.size();i++)
		ret+=a[i];
	return ret;
}

int max_score(vector<int> a)
{
	int ret=-1;
	for(int i=0;i<a.size();i++)
		if(ret<a[i])
			ret = a[i];
	return ret;
}

int findbestComb(vector<vector<int> > valid_c)
{
	if(valid_combine.size()<=0)
		return -1;
	// different kinds 
//	vector<int> kind_score(valid_c.size(),0);
//	vector<int> num_score(valid_c.size(),0);
//	vector<int> max_score(valid_c.size(),0);
	int ret=-1;
	bool istie=false;
	int index=-1;
	for(int i=0;i<valid_c.size();i++)
	{
		int kind_s = kind_score(valid_c[i]);
		int num_s = num_score(valid_c[i]);
		int max_s = max_score(valid_c[i]);
		int score = kind_s*1000 - num_s*100+max_s;
		if(ret<=score)
		{
			if(ret==score)
				istie = true;
			else 
			{
				ret = score;
				index = i;
				istie = false;
			}
		}		
	}
	if(istie)
		index = -2;
	return index;		
}

int main()
{
	types[0] = 3;
	types[1] = 1;
	types[2] = 2;
	types[3] = 0;
	
	vector<int> type_n(4,0);
	int sum=7;
	findValidCombine(sum,0,type_n);
	//displayValidComb(sum);
	int best = findbestComb(valid_combine);
	if(best==-1)
		cout<<"bad"<<endl;
	else if(best == -2)
		cout<<"tie"<<endl;
	else
	{
		cout<<"best is : "<<best<<endl;
		for(int i=0;i<valid_combine[best].size();i++)
			cout<<valid_combine[best][i]<<" ";
		cout<<endl;
	}
	return 0;
}
