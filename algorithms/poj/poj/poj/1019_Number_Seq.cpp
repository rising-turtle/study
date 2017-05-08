#include "preheader.h"
#include <cmath>
namespace{
	
	int seq_sum(int total){
		int sum=0;
		int incr=1;
		while(1){
			sum+=incr;
			if(sum>=total){
				sum-=incr;
				break;
			}
			incr++;
		}
		return total-sum;
	}
	
	int calsum(int key){
		int ret= (key*(key-1))>>1;
		return ret;
	}

	int find_seq(int total)
	{
		int upper_n = (int)sqrt((double)(2*total));
		if(calsum(upper_n)>=total){
			while(1){
				upper_n--;
				if(calsum(upper_n)<total)
					return total-upper_n;
			}		
			
		}
		else{
			while(1){
				upper_n++;
				if(calsum(upper_n)>=total)
					return total-calsum(--upper_n);
			}
		}
		return 0;

	}

};

int main1019(){
	int n;
	cin>>n;
	vector<int> tmp(n);
	for(int i=0;i<n;i++)
		cin>>tmp[i];
	for(int i=0;i<n;i++)
		cout<<find_seq(tmp[i])<<endl;
	return 0;
}


int test1019(int loc){
	cout<<"inqure loc: "<<loc<<endl;
	return find_seq(loc);
}