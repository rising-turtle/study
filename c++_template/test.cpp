#include <iostream>
#include <stdlib.h>

using namespace std;

int main()
{
	int resolution = 5;
	srand(NULL);
	for(int i=0;i<20;i++){
		float a = rand()%50 - 25;
		int idx;
		if(a<0)
			idx =(int)(a+0.5)/5.0-1;
		else
			idx = (int)(a+0.05)/5.0;
		cout<<a<<" : "<<idx<<endl;
	}
	
	return 0;
}
