#include "preheader.h"

#include <algorithm>
#include <numeric>
namespace{

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


	// 2012.10.13 下面这个策略是错误的，因为面的拼接不像线段，它有两个维度，相连的区域也可以继续拼成更大的面积，
	// 所以只是从剩下的最大的面积中切出一部分，将会造成更大的面积浪费
const static int max_res = 6;
const static int min_res = 1;
	bool consume(map<int, int>& res, vector<int >& cons, int cons_index){
		if(cons_index < 0){
			return true;
		}
		if(cons[cons_index] == 0 )
			return consume(res,cons,cons_index-1);
		if(res.size()<=0){
			return false;
		}

		// 从最大的资源块中分配资源给cons[cons_index]
		int cons_value = cons_index+1; //cons[cons_index];
		map<int,int>::reverse_iterator it_max = res.rbegin();
		while(it_max!=res.rend() && it_max->second==0){
			it_max++;
		}
		if(it_max==res.rend()) return false;
		if(it_max->first<cons_value) return false; // 资源不够了

		// 释放资源
		it_max->second--;
		cons[cons_index]--;
		// if(it_max->second==0) res.erase(it_max); // 该块资源已经完全释放
		
		// 回收剩余资源 
		int left_res =  it_max->first - cons_value;
		if(left_res > 0){
			// 资源1 (it->max-cons_value)
			if(res.find(left_res)==res.end()){
				res.insert(make_pair(left_res,1));
			}
			else res[left_res]++;
			int left_min = left_res>cons_value? cons_value:left_res;
			// int left_max = left_res>cons_value? left_res:cons_value;
			// 资源2 
			int res_num = (it_max->first-left_min)/left_min;
			if(res.find(left_min)==res.end()){
				res.insert(make_pair(left_min,res_num*2));
			}
			else
				res[left_min]+=res_num*2;
		}
		
		return consume(res,cons,cons_index);
	} 
	int find_min_res(vector<int> cons){
		// sort(cons.begin(),cons.end());
		int sum_square=0;
		for(int i=0; i<cons.size();i++){
			sum_square+=cons[i]*(i+1)*(i+1);
			//cout<<i+1<<": "<<cons[i]<<" ";
		}
		//cout<<endl;
		int least_res = sum_square/((max_res)*(max_res));
		//cout<<"least_res: "<<least_res<<"\t";

		while(1){
			// 初始化资源表
			map<int,int> res;
			vector<int> tmp_cons(cons);
			res.insert(make_pair(max_res,least_res));
			if(consume(res,tmp_cons,tmp_cons.size()-1))
				return least_res;
			least_res++;
		}
	}

	// 2012.10.13 借鉴网上的做法，直接贪心去求不同的小面积能够得到的最好的面积
	int find_min_bag1(vector<int> cons){ 
		int ret=0;
		ret+=cons[5]; // 直接加上面积为6的个数
		ret+=cons[4]; // 直接机上面积为5的个数并且减去面积1的个数
		cons[0]-=cons[4]*11;
		ret+=cons[3]; // 面积4
		int tmp=cons[3]*5;
		if(cons[1]>tmp) cons[1]-=tmp;
		else {
			cons[1]=0;
			cons[0]-=(tmp-cons[1])*4; 
		}

		// 面积3
		if(cons[2]%4==0)
			ret+=cons[2]/4;
		else{
			ret+=cons[2]/4+1;
			int tmp1 = 7-(cons[2]%4)*2;
			if(cons[1]>tmp1) {
				cons[1]-=tmp1;
				cons[0]-=36-(cons[2]%4)*9-tmp1*4; 
			}
			else{
				cons[1]=0;
				cons[0]-=36-(cons[2]%4)*9-cons[0]*4;
			}
		}
		// 面积2
		if(cons[1]>0){
			if(cons[1]%9 == 0)
				ret+=cons[1]/9;
			else{
				ret+=cons[1]/9+1;
				cons[0]-=36-(cons[1]%9)*4;
			}
			
		}
		// 面积1
		if(cons[0]>0){
			if(cons[0]%36==0)
				ret+=cons[0]/36;
			else
				ret+=cons[0]/36+1;
		}
		return ret;
	}
};


int main1017(){
	// ifstream inputf("input.txt");
	// ifstream inputf(".//inputf//1017//1017.txt");
	ifstream inputf(".//inputf//1017//1017input.txt");
	ifstream inputa(".//inputf//1017//1017A.txt");
	// ofstream ouputf("output.txt");
	// init();
	vector<int> bags(max_res);
	char buf[100];
	int answer;
	while(inputf.getline(buf,100) && inputa>>answer){
		sscanf(buf,"%d %d %d %d %d %d",&bags[0],&bags[1],&bags[2],&bags[3],&bags[4],&bags[5]);
		if(bags[0]==0 && bags[1]==0 && bags[2]==0 && bags[3]==0 && bags[4]==0 && bags[5]==0)
			break;
		cout<<find_min_bag1(bags)/*leastPacket(bags)*/<<"\t answer: "<<answer<<endl;
	}
	return 0;
}

void test1017(string file_name)
{
	ifstream inf(file_name.c_str());
	if(!inf.is_open()){
		cout<<"failed to open file: "<<file_name<<endl;
		return ;
	}
	init();
	vector<int> bags(6);
	char buf[100];
	while(inf.getline(buf,100)){
		sscanf(buf,"%d %d %d %d %d %d",&bags[0],&bags[1],&bags[2],&bags[3],&bags[4],&bags[5]);
		if(bags[0]==0 && bags[1]==0 && bags[2]==0 && bags[3]==0 && bags[4]==0 && bags[5]==0)
			break;
		cout<<leastPacket(bags)<<endl;
	}
	cout<<"succeed!"<<endl;
}