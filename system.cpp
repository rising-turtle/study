#include <iostream>
#include <stdlib.h>
#include "string.h"
#include <string>
#include <sstream>
#include <fstream>
using namespace std;

void findusb(const char* key, char* value);

int main(int argc, char* argv[]){
	// string fcmd = cmd+file;
	// system(fcmd.c_str());
	// system("dmesg | tail -f >tail.txt ");

	char value[100];
	if(argc <=1 ) {
		cout<<"no key is defined, use defalut: usb"<<endl;
		findusb("usb", value);
	}else{
		findusb(argv[1],value);
	}

	return 0;	
}

void findusb(const char* key, char* value)
{
	string cmd("dmesg | ");
	cout<<"key: "<<string(key)<<endl;

	string cmdf = cmd + string(" grep ") + string(key) + string(" | tail -f > ") + string("usb.txt");
	cout<<"cmd: "<<cmdf<<endl;
	system(cmdf.c_str());
	fstream fin("usb.txt");
	char line[4096];
	char* pv = value;
	char * pV = 0;
	while(fin.getline(line,4096)){
		// strcpy(lastline,line);
		if((pV = strstr(line, "ttyUSB")) != 0){
			strcpy(pv,pV);
		}
	}
	cout<<"value: "<<string(pv)<<endl;
}

