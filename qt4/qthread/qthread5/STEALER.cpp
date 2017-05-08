#include "STEALER.h"
#include <iostream>

using namespace std;

CStealer::CStealer():succeed(false)
{}

void CStealer::trap(int s){
	static unsigned int sleep_time = 10000;
	static int cnt = 0;
	if(s==-1) succeed = true;
	sleep_time = 2*sleep_time;

	for(unsigned int i=0;i<s;i++){
		sleep_time ++;
		//if(i==1000) cout<<"wo cacaca!"<<endl;
	}
	cout<<"get in trap "<<++cnt<<" times!"<<" sleep: "<<s<<endl;
}

void CStealer::start(){
	static int steal_time = 10000;
	int cnt =0;
	while(1){
		steal_time /=2;
		if(steal_time <200 ) steal_time = 200;
		for(int i=0;i<steal_time;i++){}
		cout<<"try to steal "<<++cnt<<" times!"<<endl;
		steal();
		if(succeed) break;
	}
	cout<<"succeed to steal the lock! HAHA!"<<endl;
}
