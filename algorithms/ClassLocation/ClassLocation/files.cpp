#include <iostream>
#include <fstream>

using namespace std;

class CC{
public:
	CC(){}
	~CC(){
		if(file_handle.is_open())
			file_handle.close();
	}
public:
	void getfile(){
		file_handle.open("D:\\D.txt",ios::out|ios::app);
		file_handle<<"ca"<<endl;
	}
	fstream file_handle;
};


void testfile()
{
 ofstream logfile;
 logfile.open("D:\\11.log",ios::app); // 
 if(logfile.is_open()){
	 logfile<<"succeed!"<<endl;
	 logfile<<"succeed!"<<endl;
	 logfile<<"succeed!"<<endl;
	 logfile.close();
 }
}
//void main(){
//	testfile();
//	CC f;
//	getchar();
//}