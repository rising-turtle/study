#include <iostream>
#include <vector>
#include <stdlib.h>

using namespace std;

#define T 10485760

struct MEM{
	~MEM(){
	cout<<"destructor is called!"<<endl;
}
	int m[T];
};


int main()
{
	vector<struct MEM*> pm;
	cout<<"before allocate!"<<endl;
	system("top -n 1| grep Mem| cut -d \",\" -f 2 >mem.txt");
	struct MEM* p = new struct MEM;
	pm.push_back(p);
	/*for(int i=0;i<1;i++)
	{
		pm.push_back(new struct MEM);
	}*/
	cout<<"after allocate!"<<endl;
	system("top -n 1| grep Mem| cut -d \",\" -f 2 >>mem.txt");
	vector<struct MEM*>::iterator it = pm.begin();
	while(it!=pm.end()){
		it = pm.erase(it);
	}
	cout<<"after erase!"<<endl;
	system("top -n 1| grep Mem| cut -d \",\" -f 2 >>mem.txt");
	delete p;
	return 0;
}
