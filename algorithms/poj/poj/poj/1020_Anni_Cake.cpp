#include "preheader.h"

namespace {
	map<int,int> res;
	void init(){
		res.insert(make_pair(6,6));
	}

	bool consumeBag(int side){
		bool addBag = false;
		map<int,int>::reverse_iterator it_max = res.rbegin();
		if(res.size()==0 || side>it_max->first)
		{
			addBag = true;
			if(side<6){
				res.insert(make_pair(6-side,6));
				int low = side < 6-side?side:6-side;
				int high = side >= 6-side?side:6-side;
				res.insert(make_pair(low,high));
			}
		}
		else{
			for(map<int,int>::iterator it=res.begin();it!=res.end();it++){
				if(it->first>=side){
					int low = it->first;
					int high = it->second;
					res.erase(it);
					if(low!=side)
						res.insert(make_pair(low-side,high));
					low = side < high-side?side:high-side;
					high = side >= high-side?side:high-side;
					if(low>0)
						res.insert(make_pair(low,high));
					break;
				}
			}
		}
		return addBag;
	}

	int leastPacket(vector<int>& bags){
		int ret = 0;
		int side = 6;
		for(int i=bags.size()-1;i>=0;){
			if(bags[i]==0){
				side--;
				i--;
				continue;
			}
			if(consumeBag(side)){
				ret++;
			}
			bags[i]--;
		}
		return ret;
	}

};

int test1020(){
	int n;
	cin>>n;
	for(int i=0;i<n;i++)
	{
		res.clear();
		int total;
		cin>>total;
		res.insert(make_pair(total,total));
		int n_pieces;
		cin>>n_pieces;
		for(int j=0;j<n_pieces;j++){
			int side;
			cin>>side;
		}
	}
	return 0;
}