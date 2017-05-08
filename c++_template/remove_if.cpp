#include <iostream>
#include <algorithm>
#include <vector>
#include <stdlib.h>

#define N 100

using namespace std;
int gen(){
	static bool bfirst = true;
	if(bfirst){
		bfirst = false;
		srand(NULL);
	}
	return (rand()%N);
}

bool remove( int a){
	return (a>50);
}

void display(vector<int>& t){
	for(int i=0;i<t.size();i++)
		cout<<t[i]<<" ";
	cout<<endl;
}
int main()
{
	vector<int > t(10);
	// vector<int > p(10);
	generate(t.begin(),t.end(),gen);
	display(t);
	// remove_if(t.begin(),t.end(),(bool(*)(int))(remove));
	// remove_copy_if(t.begin(),t.end(),p.begin(),(bool(*)(int))(remove));
	vector<int>::iterator it = remove_if(t.begin(),t.end(),(bool(*)(int))(remove));
	t.erase(it,t.end());
	display(t);
	cout<<t.size()<<endl;
	// display(p);
	return 0;
}
