#include <iostream>
#include <string>
#include <vector>

using namespace std;

int minPossibleValue(vector<int> pieces,int& sum_out)
{
	int sum =0;
	int max = -1;
	for(int i=0;i<pieces.size();i++)
	{
		sum+=pieces[i];
		if(max<pieces[i])
			max=pieces[i];
	}
	sum_out = sum;
	return (int)(sum/max);
}

bool IsDivided(vector<int> pieces, int divide, int sum)
{
	int rest = sum;
	int single_rest;
	vector<bool> used(pieces.size(),false);
	while(1)
	{
		single_rest = divide;
		int k=0;
		while(single_rest>0 && k<pieces.size())
		{
			if(!used[k] && pieces[k]<single_rest)
			{
				used[k]=true;
				single_rest -=pieces[k]; 
			}
		    k++;
		}
		if(single_rest == 0)
		{
			rest -=divide;
		}
		else
			return false;
		if(rest==0)
			return true;
		if(rest<0)
			break;
	}	
	return false;
}


int findMinStick(vector<int> pieces)
{
	int sum;
	int divide_n = minPossibleValue(pieces,sum);
	cout<<"sum: "<<sum<<endl;
	for(int j=divide_n;j>=2;j--)
	{		
		if(sum%j!=0) continue;
		int divide = sum/j;
		cout<<"divide: "<<divide<<endl;
		if(IsDivided(pieces,divide,sum));
			return divide;
	}
	return -1;
}

int main()
{
	int tem[9] = {5,2,1,5,2,1,5,2,1};
	int tem2[4] = {1,2,3,4};
	vector<int> temp;
	for(int i=0;i<(sizeof(tem2)/sizeof(int));i++)
		temp.push_back(tem2[i]);
	cout<<findMinStick(temp)<<endl;
	return 0;
}
