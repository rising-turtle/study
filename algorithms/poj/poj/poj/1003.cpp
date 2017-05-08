

/************************************************************************/
/* 当计算连续的数值时，比如求和，求积等，不要每次都去计算，应该利用查表的
/* 方法，构建 查找表，然后二分查找
/************************************************************************/

#include <iostream>
#include <vector>
#include <stdlib.h>
#include <ctime>

using namespace std;
namespace{
vector<float> _score;

#define max_range 5.20
#define min_range 0.01

void init(){
	int k=2;
	float score=(float)(1.0/(float)k);
	_score.push_back(0);
	while(1){
		if(score>5.20) break;
		_score.push_back(score);
		k++;
		score+=(float)(1.0/(float)k);
	}
}

int cards1(float range)
{
	for(int i=1;i<_score.size();i++)
	{
		if(_score[i]>=range)
			return i;
	}
}

int cards(float range)
{
	int min_ = 1;
	int max_ = _score.size()-1;
	while(1){
		if(min_>=max_) return min_;
		int mid = min_ + (max_-min_)/2;
		if(_score[mid]>=range){
			if(mid==min_ || _score[mid-1]<range)
				return mid;
			max_ = mid-1;
			continue;
		}
		else{
			if(mid==max_ || _score[mid+1]>=range)
				return mid+1;
			min_ = mid+1;
			continue;
		}
	}
	// impossible
	return -1;
}

void read(){
	float range;
	while(cin>>range){
		if(range<min_range || range>max_range)
			break;
		cout<<cards(range)<<" card(s)"<<endl;
	}
	return ;
}

void t1003(){
	init();
	read();
}

extern "C" int mycards(double range);

void testmycards(){
	init();
	double range;
	srand((unsigned int)(time(NULL)));
	for(int i=0;i<1000;i++){
		range = (rand()%5000)/(double)(1000)+0.01;
		if(1 || cards(range)!=mycards(range)){
			cout<<"range= "<<range<<" cards: "<<cards(range)<<" mycards: "<<mycards(range)<<endl;
		}
	}
	cout<<"succeed!"<<endl;
}

};