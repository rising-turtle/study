#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <stdlib.h>

int getMemAvailable(){
	char data[1024]; 
	FILE* pipe; 
	pipe = popen("free --kilo", "r"); 
	int line = 1;
	char* pre_token = NULL; 

	int ret = 0;

	while(fgets(data, sizeof(data), pipe) != NULL){

		printf("line %d: %s ", line++, data); 
		char* token = strtok(data, " "); 
		if(line >= 4) break; 
		while(token !=  NULL){
			pre_token = token; 
			// printf("%s\n", token);
			token = strtok(NULL, " ");
		}

	}
	if(pre_token != NULL){
		ret = atoi(pre_token); 
	}
	// printf("pre_token = %s and ret = %d\n", pre_token, ret);
	// close(*pipe);
	fclose(pipe);
	return ret;
}

int test_func(){
	printf("in test func call memavailable: %d\n ", getMemAvailable());
}

int main(int argc, char* argv[]){

	printf("before func call memavailable: %d\n ", getMemAvailable());
	test_func();
	printf("after func call memavailable: %d\n ", getMemAvailable());
	sleep(30);

	return 0;

}
