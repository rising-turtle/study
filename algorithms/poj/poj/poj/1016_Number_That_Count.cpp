#include "preheader.h"


// 题中所需要的是找到循环的距离，即并不需要回到初始的循环状态，
// 但是懒得改了，本方法就是看能否回到初始的循环状态，不能就返回失败

namespace{

 string firststatus;
 set<string> prestatus; 

bool calNextStatus(vector<int>& pre, int& turns)
{
	map<int,int> record;
	for(int i=0;i<pre.size();i++){
		int key = pre[i];
		if(record.find(key)==record.end()){
			record.insert(make_pair(key,1));
		}
		else
			record[key]++;
	}
	vector<int> tmp;
	for(map<int,int>::iterator it=record.begin();it!=record.end();it++){
		tmp.push_back(it->second);
		tmp.push_back(it->first);
	}
	pre.swap(tmp);
	
	stringstream str;
	string cur_str;
	for(int i=0;i<pre.size();i++)
		str<<pre[i];
	str>>cur_str;
	cout<<"in loop "<<turns<<" : "<<cur_str<<endl;
	if(firststatus==cur_str)
		return true;
	if(prestatus.find(cur_str)!=prestatus.end())
		return false;
	prestatus.insert(cur_str);
	return calNextStatus(pre,++turns);
}

void display(vector<int>& v){
	for(int i=0;i<v.size();i++)
		cout<<v[i];
	cout<<endl;
}
void init(vector<int>& v){
	// 314213241519
	// 111222234459 
	//int input[] = {3,1,4,2,1,3,2,4,1,5,1,9};
	int input[] = {1,1,1,2,2,2,2,3,4,4,5,9};
	stringstream strs;
	for(int i=0;i<sizeof(input)/sizeof(int);i++)
	{
		v.push_back(input[i]);
		strs<<input[i];
	}
	strs>>firststatus;
}


};

void test1016()
{
	vector<int> pre;
	init(pre);
	int turns=1;
	if(calNextStatus(pre,turns)){
		cout<<firststatus<<" is self ok in loop "<<turns<<endl;
	}
	else{
		cout<<"failed!"<<endl;
	}
	//display(pre);
}

namespace {

#include <map>
#include <set>
	using std::map;
	using std::set;

	string myatoi(int i){
		string ret;
		while(i>0){
			ret = (char)(i%10 + '0') + ret;
			i/=10;
		}
		return ret;
	}

	string calLoopNum(string str){
		string ori(str);
		string ret(str);
		int loop=0;
		int record[10];

		stringstream strret;
	
		map<string, int> pre;
		pre.insert(make_pair(ret,loop));

		while(loop<15){	
			loop++;
			string tmp;
			memset(record,0,sizeof(int)*10);
			for(int i=0;i<ret.size();i++){
				record[ret[i]-'0']++;
			}
			for(int i=0;i<10;i++){
				if(record[i]>0)
				{
					tmp+= myatoi(record[i]);//record[i]+'0';
					tmp+=i+'0';
				}
			}
			map<string, int>::iterator it = pre.find(tmp);
			if(it==pre.end()){
				pre.insert(make_pair(tmp,loop));
				//cout<<"loop "<<loop<<" : "<<tmp<<endl;
				ret = tmp;
				continue;
			}
			int step = it->second ;
			if(step == 0 && loop == 1)
					strret<<ori<<" is self-inventorying";
			else if(step==pre.size()-1)
					strret<<ori<<" is self-inventorying after "<<step<< " steps";
			else{
				int loopl = pre.size() - step;
				strret<<ori<<" enters an inventory loop of length "<<loopl;
			}
			return strret.str();
		}
		strret<<ori<<" can not be classified after 15 iterations";
		return strret.str();
	}
}

void testDataSet()
{
	ifstream input("D:\\myproj\\poj\\data\\1016\\1016input.txt");
	ifstream output("D:\\myproj\\poj\\data\\1016\\1016output.txt");
	
	string inputs;
	string outputs;
	int cnt = 0;
		
	char line[8192];

	while(output.getline(line,8192)){
		input>>inputs;
		outputs = string(line);
		cnt++;
		string calout = calLoopNum(inputs);//otheralgo(inputs);
		if(outputs != calout)
		{
			cout<<"output should be : "<<outputs<<endl;
			cout<<"calput actual is: "<<calout<<endl;
			cout<<"error in "<<cnt<<" item!"<<endl;
		}
		cout<<"frame: "<<cnt<<"succeed!"<<endl;
	}
	cout<<"succeed!"<<endl;
}


int main1016()
{
	/*char line[8192];
	while(cin>>line){
		if(strcmp(line,"-1")==0)
			break;
		calLoopNum(string(line));
	}*/
	
	// test 
	//calLoopNum("31123314");	// is self-inventorying 
	//calLoopNum("111222234459");	// enters an inventory loop of length 2
	//calLoopNum("314213241519"); // enters an inventory loop of length 2
	//calLoopNum("21221314"); // is self-inventorying after 2 steps

	testDataSet();

	return 0;
}