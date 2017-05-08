#include <iostream>
#include "stdio.h"
#include <string.h>

using namespace std;

int main()
{
	char line[] = "yade, wo de ge xin; na !";
	char delim[] = ",;";
	cout<<"line: "<<line<<endl;
	char *p = strtok(line,delim);
	if(p) printf("%s\n",p);
	while(1){
		p = strtok(NULL, delim);
		if(!p) break;
		printf("%s , %s\n",line,p);
	}
	
	return 0;
}
