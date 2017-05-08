#include <iostream>
#include <list>
#include "stdlib.h"
#include <ctime>

using namespace std;

size_t avr_num = 10;
list<float> quit_err;
float sum_err = 0;
float avr_err = 0;

void seterror(float err){
	if(quit_err.size() == 0)
	{
		quit_err.push_back(err);
		sum_err = err;
		avr_err = err;
		return ;
	}
	bool inserted = false;
	for(list<float>::iterator it = quit_err.begin(); it!= quit_err.end(); it++){
		if( *it > err)
		{
			inserted = true;
			quit_err.insert(it,err);
			break;
		}
	}


	if(!inserted){
		quit_err.push_back(err);
	}

	if(quit_err.size() == avr_num + 1) // remove the last value
	{
		if(inserted) // nothing changed
		{
			sum_err = sum_err - *(quit_err.rbegin()) + err;
			avr_err = sum_err / (float)(avr_num);
		}
		//quit_err.erase(quit_err.rbegin());
		quit_err.pop_back();
		int a = quit_err.size();
		return ;
	}
	else{
		sum_err += err;
		avr_err = sum_err / (float)(quit_err.size());
	}
	return ;
}

ostream& operator <<(ostream& out, list<float>& p){
	for(list<float>::iterator it=p.begin();it!=p.end(); it++)
		out<<*it<<" ";
	out<<endl;
	out<<"num : "<<p.size()<<endl;
	out<<"sum : "<< sum_err<<endl;
	out<<"avr : "<<avr_err<<endl;
	return out;
}

void generatelist(size_t n){
	srand(unsigned(time(NULL)));
	for(size_t i=0; i< n; i++){
		float r = (rand()%15)*0.1;
		seterror(r);
	}
}

class a{
public:
	const int c;
	a():c(8){
		c = 8;
	}
};

void main()
{
	//generatelist(20);
	//cout<<quit_err;

	int ll;
	aa.change();
	//cout<<aa.c<<endl;
	/*list<float> q(10,0);
	cout<<q.size()<<endl;
	list<float>::iterator it = q.begin();
	q.insert(it,2);
	cout<<q.size()<<endl;*/

	getchar();
	return;
}